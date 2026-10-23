import laspy
import open3d as o3d
import random
import os
from collada import *
import numpy as np
import collada
import rasterio
from rasterio.windows import Window
from PIL import Image, ImageFilter
import math
import quaternion
from math import cos, sin, acos
from scipy.linalg import lstsq
import matplotlib.pyplot as plt

Image.MAX_IMAGE_PIXELS = None

SIZE_AREA = 12000       # tamanho do mapa
PIXELS_AREA_HERD = 60  # tamanho da area para a manada (pixels)
PIXELS_AREA_ANIMAL = 50  # tamanho da area para cada animal na manada (cm)
XMIN, YMIN = 0, 0       # limites
MIN_DIST_ANIMAL_X = 1.8 # Distancia minima de x para outros bois
MIN_DIST_ANIMAL_Y = 3.3 # Distancia minima de y para outros bois   

super_animal = [] #Array com todos os bois para checagem de sobreposição

import numpy as np
import matplotlib.pyplot as plt
from math import cos, sin

def quaternion_multiply(q1, q2):
    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2
    return np.array([
        w1*w2 - x1*x2 - y1*y2 - z1*z2,
        w1*x2 + x1*w2 + y1*z2 - z1*y2,
        w1*y2 - x1*z2 + y1*w2 + z1*x2,
        w1*z2 + x1*y2 - y1*x2 + z1*w2
    ])

def calculate_roll_pitch(points, cont, epsilon=1e-6):
    X = points[:, 0]
    Y = points[:, 1]
    Z = points[:, 2]
    
    # Matriz de design para o ajuste do plano
    A = np.c_[X, Y, np.ones(X.shape)]
    
    # Resolve os mínimos quadrados
    coeffs, _, _, _ = np.linalg.lstsq(A, Z, rcond=None)
    
    # Coeficientes
    a, b, c = coeffs
    print(f"Coeficientes do plano ajustado: a= {a}, b= {b}, c= {c}\n")

    # Normal do plano
    normal = np.array([-a, -b, 1])
    normal_normalized = normal / np.linalg.norm(normal)
    print(f"Normal: {normal}\n")
    print(f"Normal_Normalized: {normal_normalized}")

    # Vetor v1 (eixo Z)
    v1 = np.array([0, 0, 1])

    # Produto escalar entre os vetores normalizados
    dot_product = np.dot(normal_normalized, v1)
    
    # Evita erros numéricos garantindo que dot_product esteja no intervalo [-1,1]
    dot_product = np.clip(dot_product, -1.0, 1.0)

    # Ângulo em radianos
    theta = -np.arccos(dot_product)

    # Se o ângulo for muito pequeno, retorna o quaternion de identidade
    if abs(theta) < epsilon:
        print("Ângulo muito pequeno, ignorando rotação.")
        return np.array([1, 0, 0, 0])

    theta_degrees = np.degrees(theta)
    print(f'Ângulo em graus: {theta_degrees}')
    print(f"Ângulo em radianos: {theta}")

    # Vetor v2 (eixo de rotação, perpendicular a v1 e à normal)
    v2 = np.cross(normal_normalized, v1)
    v2_normalized = v2 / np.linalg.norm(v2)
    print(f"\nVetor v2: {v2_normalized}")

    # Calculando o quaternion
    w = cos(theta / 2)
    x = v2_normalized[0] * sin(theta / 2)
    y = v2_normalized[1] * sin(theta / 2)
    z = v2_normalized[2] * sin(theta / 2)

    quaternion = np.array([w, x, y, z])
    print(f"Quaternion: {quaternion}")
    print("================================\n")

    return quaternion

def calculate_mesh_height(bull_mesh):
    vertices = []
    normals = []
    texcoords = []
    indices = []

    #Carregar a mesh original do animal
    #bull_mesh_path = "/catkin_ws/src/animal-insertion/media/"
    bull_mesh_path = "../../animal-insertion/media/"
    mesh = Collada(bull_mesh_path + bull_mesh)

    #Iterar pelas geometrias no arquivo
    for geom in mesh.geometries:
        for primitive in geom.primitives:
            #extrai os vertices
            if isinstance(primitive, collada.triangleset.TriangleSet):
                vertices.extend(primitive.vertex)

                #extrai as normais
                if primitive.normal is not None:
                    normals.extend(primitive.normal)

                #extrai as coordenadas de textura   
                if primitive.texcoordset is not None and len(primitive.texcoordset) > 0:
                    texcoords.extend(primitive.texcoordset[0])

                #extrai os indices
                indices.extend(primitive.indices)

    #atribuicao dos vertices
    vertices = np.array(vertices)

    # Garantir que os vértices são organizados como (x, y, z)
    vertices = vertices.reshape(-1, 3)  

    # Encontrar o menor valor de Z
    min_z = np.min(vertices[:, 2]) 
    print(f"O menor valor de Z é: {min_z}")

    return min_z


def generate_coordinates_herd(xmin_pixel_herd, xmax_pixel_herd, ymin_pixel_herd, ymax_pixel_herd, tif_image_path, las_file, x_mean_fit, y_mean_fit, z_mean_fit, 
z_axis_sum, num_animals_herd, i, sigmas, models, yaw_mean, yaw_sigma, out_dir_path):
    cont = 1

    #pontos dentro da manada
    points_in_herd = []

    #Centros da distribuição Gaussiana
    x_center = (xmin_pixel_herd + xmax_pixel_herd)/2
    y_center = (ymin_pixel_herd + ymax_pixel_herd)/2

    #Distribuição padrao do gaussiano (quanto maior o valor mais separados)
    sigma = sigmas[i]
    print(f"SIGMA DE I={i} : {sigma}")

    # print(f" Coordenada x_center: {x_center}")
    # print(f" Coordenada y_center: {y_center}")
    
    for k in range(num_animals_herd[i]):
        while True:
            contador = 0
            # Gere coordenadas (x, y) gaussianas ao redor do centro
            x = int(np.random.normal(loc=x_center, scale=sigma))
            y = int(np.random.normal(loc=y_center, scale=sigma))

            #Checagem se o animal esta dentro da area para manada
            if xmin_pixel_herd < x <= xmax_pixel_herd and ymin_pixel_herd < y < ymax_pixel_herd:
                
                print(f"Animal {k + 1}/{num_animals_herd[i]} Dentro da area do rebanho: x={x}, y={y}")
                xmin_pixel_animal = x - PIXELS_AREA_ANIMAL
                xmax_pixel_animal = x + PIXELS_AREA_ANIMAL
                ymin_pixel_animal = y - PIXELS_AREA_ANIMAL
                ymax_pixel_animal = y + PIXELS_AREA_ANIMAL

                #Abre a imagem georreferenciada
                with rasterio.open(tif_image_path) as src:
                    window = Window(xmin_pixel_animal, ymin_pixel_animal, xmax_pixel_animal, ymax_pixel_animal)
                    
                    #seta as transformacoes para as coordenadas
                    clipped_img = src.read(window=window)
                    clipped_transform = src.window_transform(window)

                #define os limites para o animal
                xmin_clipp, ymin_clipp = clipped_transform * (0, 0)
                xmax_clipp, ymax_clipp = clipped_transform * (PIXELS_AREA_ANIMAL , PIXELS_AREA_ANIMAL )

                if xmin_clipp > xmax_clipp:
                    xmin_clipp, xmax_clipp = xmax_clipp, xmin_clipp

                if ymin_clipp > ymax_clipp:
                    ymin_clipp, ymax_clipp = ymax_clipp, ymin_clipp

                print(f"Recorte nas coordenadas: {xmin_clipp}, {ymin_clipp}, {xmax_clipp}, {ymax_clipp}")

                output_file = os.path.join(out_dir_path, "las_cortado_animal.las")
                
                #Comando las2las com verificação de sucesso
                result = os.system(f"las2las64 -i {las_file} -o {output_file} -keep_xy {xmin_clipp} {ymin_clipp} {xmax_clipp} {ymax_clipp}")
                print(f"Comando las2las retornou: {result}")
                
                # Verificar se o arquivo foi criado
                if os.path.exists(output_file):
                    print("Arquivo LAS criado com sucesso!")
                else:
                    print("Erro ao criar o arquivo LAS.")
                    break
                    continue
            
                main_las_file_path_fit = output_file

                las_file_fit = laspy.read(main_las_file_path_fit)

                #ADICIONADO 
                os.remove(output_file)

                points_las_fit = np.vstack((las_file_fit.x, las_file_fit.y, las_file_fit.z)).transpose()

                points_las_fit = points_las_fit - np.array([x_mean_fit, y_mean_fit, z_mean_fit])

                points_las_fit = points_las_fit[np.isfinite(points_las_fit).all(axis=1)]
                print(f"Numero de pontos dentro da area: {points_las_fit.shape[0]}")

                nuvem_pontos_fit = o3d.geometry.PointCloud()
                nuvem_pontos_fit.points = o3d.utility.Vector3dVector(points_las_fit)

                pixel_meio_z = np.mean(points_las_fit[:, 2]) 
                pixel_meio_x = (xmax_clipp + xmin_clipp) / 2 - x_mean_fit
                pixel_meio_y = (ymax_clipp + ymin_clipp) / 2 - y_mean_fit

                print(f"coordenada de spawn X: {pixel_meio_x}")
                print(f"coordenada de spawn Y: {pixel_meio_y}")
                print(f"coordenada de spawn Z: {pixel_meio_z}\n")
                
                #subtração para tirar deslocamento no eixo z
                pixel_meio_z = pixel_meio_z - z_axis_sum
                #print(f"A média da coordenada Z é: {pixel_meio_z}")

                # Verificar distância mínima de outros bois
                too_close = False
                for existing_point in super_animal:
                    distance_x = existing_point[0] - pixel_meio_x
                    distance_y = existing_point[1] - pixel_meio_y  
                    if abs(distance_x) < MIN_DIST_ANIMAL_X and abs(distance_y) < MIN_DIST_ANIMAL_Y:
                        too_close = True
                        break
                
                if too_close:
                    print(f"Bois sobrepostos, gerando nova posição para o animal {k + 1}.")
                    continue
                else:
                    # Salvar coordenada válida
                    #Funcao para calcular o roll e pitch
                    rp_quaternion = calculate_roll_pitch(points_las_fit,cont)

                    # Gerando um yaw aleatório para o boi dentro da distribuição normal
                    yaw_angle = np.random.normal(loc=yaw_mean, scale=yaw_sigma)

                    # Converter yaw para quaternion
                    yaw_quaternion = np.array([
                        np.cos(yaw_angle / 2),
                        0,
                        0,
                        np.sin(yaw_angle / 2)
                    ])

                    # Combinar com o quaternion existente (roll, pitch)
                    quaternion = quaternion_multiply(rp_quaternion, yaw_quaternion)

                    cont = cont + 1 
                    points_in_herd.append((pixel_meio_x, pixel_meio_y, pixel_meio_z, quaternion[0] ,quaternion[1] , quaternion[2], quaternion[3]))
                    super_animal.append((pixel_meio_x, pixel_meio_y, pixel_meio_z, quaternion[0] ,quaternion[1] , quaternion[2], quaternion[3]))
                    break
    
    # Salvar coordenadas no formato necessário
    with open(os.path.join(out_dir_path, "coordinates.txt"), "a") as arquivo:
        for coord in points_in_herd:
            arquivo.write(f"{coord[0]}, {coord[1]}, {coord[2]}, {coord[3]}, {coord[4]}, {coord[5]}, {coord[6]}, {models[i]}\n")
    print("Coordinates saved in 'coordinates.txt'")

def area_cutting_herd(imagem_semantica_path, las_path, tif_image_path, models, num_animals_herd, num_herds, sigmas, out_dir_path, x_mean_fit, y_mean_fit, z_mean_fit): 
    #laço ate o numero de manadas
    for i in range(len(num_herds)):
        cont = 0    
        while cont < int(num_herds[i]):
            print("\n================================")
            print(f"Procurando area para manada n:{cont+1}/{int(num_herds[i])}")  

            #ADICIONADO

            las_header = laspy.read(las_path).header
            with rasterio.open(tif_image_path) as src:
                col_min, row_min = (~src.transform) * (las_header.mins[0], las_header.mins[1])
                col_max, row_max = (~src.transform) * (las_header.maxs[0], las_header.maxs[1])

            # Calcula os limites com base no LAS
            lim_i_min = int(min(col_min, col_max)) + PIXELS_AREA_HERD
            lim_i_max = int(max(col_min, col_max)) - PIXELS_AREA_HERD
            lim_j_min = int(min(row_min, row_max)) + PIXELS_AREA_HERD
            lim_j_max = int(max(row_min, row_max)) - PIXELS_AREA_HERD

            # Trava os limites dentro do tamanho da imagem
            lim_i_min = max(lim_i_min, PIXELS_AREA_HERD)
            lim_i_max = min(lim_i_max, SIZE_AREA - PIXELS_AREA_HERD - 1)
            lim_j_min = max(lim_j_min, PIXELS_AREA_HERD)
            lim_j_max = min(lim_j_max, SIZE_AREA - PIXELS_AREA_HERD - 1)

            print(f"O tamanho total da imagem é 12000x12000.")
            print(f"Devido ao tamanho do .las, os bois só podem nascer no limite:")
            print(f"X: {lim_i_min} até {lim_i_max} | Y: {lim_j_min} até {lim_j_max}")

            while True:
                #seta um numero aleatorio (i,j) para a imgem semantica

                #random_pixel_i_herd = random.randint(PIXELS_AREA_HERD, SIZE_AREA - PIXELS_AREA_HERD)
                #random_pixel_j_herd = random.randint(PIXELS_AREA_HERD, SIZE_AREA - PIXELS_AREA_HERD)

                #ADICIONADO
                random_pixel_i_herd = random.randint(lim_i_min, lim_i_max)
                random_pixel_j_herd = random.randint(lim_j_min, lim_j_max)

                #carrega a imagem semantica
                im = Image.open(imagem_semantica_path) 
                im_matriz = im.load()
                
                #carrega a cor desse pixel aleatorio
                cor_pixel_semantica = im_matriz[random_pixel_i_herd, random_pixel_j_herd]
                #print(f"Cor do pixel semântico: {cor_pixel_semantica}")

                contador = 0
                #verifica se o pixel é mato baixo
                if (cor_pixel_semantica[0] == 2) and (cor_pixel_semantica[1] == 2) and (cor_pixel_semantica[2] == 2):
                    area_invalida = False

                    #verifica a area em volta do pixel escolhido
                    for k in range(random_pixel_i_herd - PIXELS_AREA_HERD, random_pixel_i_herd + PIXELS_AREA_HERD):
                        if area_invalida:
                            break 
                        for j in range(random_pixel_j_herd - PIXELS_AREA_HERD, random_pixel_j_herd + PIXELS_AREA_HERD):
                            cor_pixel_atual = im_matriz[k, j]

                            #se for de outra cor ele cancela
                            if (cor_pixel_atual[0] != 2) and (cor_pixel_atual[1] != 2) and (cor_pixel_atual[2] != 2):
                                #print(f"Pixel fora da área: {cor_pixel_atual}")
                                #print("Área não aceita")
                                area_invalida = True  
                                break 
                            
                            #se for da mesma cor ele aumenta o contador
                            else:
                                #print(f"Pixel dentro da área: {cor_pixel_atual}")
                                contador += 1

                #verifica se o contador tem o mesmo valor da area em volta do pixel
                if contador == (PIXELS_AREA_HERD * 2) * (PIXELS_AREA_HERD * 2):
                    print(f"Area Encontrada!\n")

                    #calcula os pixels xmin, xmax, ymin, ymax da manada
                    xmin_pixel_herd = random_pixel_i_herd - PIXELS_AREA_HERD
                    ymin_pixel_herd = random_pixel_j_herd - PIXELS_AREA_HERD
                    xmax_pixel_herd = random_pixel_i_herd + PIXELS_AREA_HERD
                    ymax_pixel_herd = random_pixel_j_herd + PIXELS_AREA_HERD

                    #tamanho da area
                    size = (PIXELS_AREA_HERD * 2) + 1

                    #Abre a imagem georreferenciada
                    with rasterio.open(tif_image_path) as src:
                        window = Window(xmin_pixel_herd, ymin_pixel_herd, SIZE_AREA, SIZE_AREA)
                        
                        #seta as transformacoes para as coordenadas
                        clipped_img = src.read(window=window)
                        clipped_transform = src.window_transform(window)

                    #funcao para calcular a altura da mesh para somar ao eixo z
                    print(f"i: {i}")
                    z_axis_sum = calculate_mesh_height(models[i]) 

                    # Gerando um ângulo médio de yaw para a manada (aleatório entre 0 e 360 graus)
                    yaw_mean = np.random.uniform(0, 2 * np.pi)  # em radianos
                    yaw_sigma = np.radians(20)  # Variação de ±30 graus

                    #Funcao para achar as coordenadas dos bois na manada
                    generate_coordinates_herd(xmin_pixel_herd, xmax_pixel_herd, ymin_pixel_herd, ymax_pixel_herd, tif_image_path, las_path, x_mean_fit, y_mean_fit, 
                    z_mean_fit, z_axis_sum, num_animals_herd, i, sigmas, models, yaw_mean, yaw_sigma, out_dir_path)

                    cont = cont + 1

                    break

#Calculo do deslocamento do mapa
def set_displacement_limits(las_path, tif_image_path):

    with rasterio.open(tif_image_path) as src:
        #Definir a area de corte (xmin, ymin, xmax, ymax)
        window = Window(XMIN, YMIN, SIZE_AREA, SIZE_AREA)
        
        #Ler a parte da imagem dentro da janela
        clipped_img = src.read(window=window)
        
        #Obter a transformacao para a parte cortada da imagem
        clipped_transform = src.window_transform(window)
    
    xmin, ymin = clipped_transform * (0, 0)
    xmax, ymax = clipped_transform * (SIZE_AREA ,SIZE_AREA)

    las_file = laspy.read(las_path)
    
    points = np.vstack((las_file.x, las_file.y, las_file.z)).transpose()

    xmin_clipp = xmin
    ymin_clipp = ymin
    xmax_clipp = xmax
    ymax_clipp = ymax

    if xmin_clipp > xmax_clipp:
        temp = xmin_clipp
        xmin_clipp = xmax_clipp
        xmax_clipp = temp 

    if ymin_clipp > ymax_clipp:
        temp = ymin_clipp
        ymin_clipp = ymax_clipp
        ymax_clipp = temp 

    #Filtrar os pontos dentro da area desejada
    indices = np.where((points[:, 0] >= xmin_clipp) & (points[:, 0] <= xmax_clipp) & (points[:, 1] >= ymin_clipp) & (points[:, 1] <= ymax_clipp))

    #Selecionar apenas os pontos dentro da area desejada
    points_inside_area = points[indices]

    #Subtracao dos pontos para tirar deslocamento
    x_mean = np.mean(points_inside_area[:,0])
    y_mean = np.mean(points_inside_area[:,1])
    z_mean = np.mean(points_inside_area[:,2])

    #x_mean = 2498502.0410002535
    #y_mean = 1118501.938342049

    return x_mean, y_mean, z_mean

if __name__ == "__main__":
    import sys
    
    # Nome do arquivo
    filename = "arguments.txt"

    # Inicializando os vetores
    models = []         # Vetor para os modelos
    num_herds = []  # Vetor para o número de manadas
    num_animals_herd = []     # Vetor para os bois por manada
    sigmas = []          # Vetor para os sigmas

    # Lendo o arquivo
    with open(filename, "r") as file:
        grupo = []
        for line in file:
            line = line.strip()  
            if line == "":  
                if grupo:  
                    models.append(grupo[0])
                    num_herds.append(int(grupo[1]))
                    num_animals_herd.append(int(grupo[2]))
                    sigmas.append(float(grupo[3]))
                    grupo = []
            else:
                grupo.append(line)  

        if grupo:
            models.append(grupo[0])
            num_herds.append(int(grupo[1]))
            num_animals_herd.append(int(grupo[2]))
            sigmas.append(float(grupo[3]))

    if len(sys.argv) != 5:
        print("Uso: python animal-insertion.py /tif_image /las_file /semantic_texture path_out")
        sys.exit(1)
    elif len(sys.argv) == 5:
        tif_image_path = sys.argv[1]
        las_path = sys.argv[2]
        original_semantic_texture = sys.argv[3]
        out_dir_path = sys.argv[4]

    x_mean,y_mean,z_mean = set_displacement_limits(las_path, tif_image_path)
    area_cutting_herd(original_semantic_texture, las_path, tif_image_path, models, num_animals_herd, num_herds, sigmas, out_dir_path, x_mean, y_mean, z_mean)
 