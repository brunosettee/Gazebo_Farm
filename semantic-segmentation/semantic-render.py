import os
import sys
import numpy as np
from collada import *
from PIL import  Image, ImageFilter
from OpenGL.GL import *
from glfw.GLFW import *
from glfw import _GLFWwindow as GLFWwindow
import glm
from shader_s import Shader
import platform, ctypes
import math
import re
import numpy as np
import matplotlib.pyplot as plt

Image.MAX_IMAGE_PIXELS = None

def calculate_fov_y(fy,height):
    fov_y = 2 * np.arctan(height / (2 * fy))
    return fov_y

def processar_arquivo_txt(caminho_arquivo):
    dados = {}
    lendo = 0 #0: Não Lendo, 1: Lendo gps gt, 2: lendo gps, 3: lendo orientacao, 4: lendo pose
    lidos = 0

    with open(caminho_arquivo, 'r') as f:
        linhas = f.readlines()

        # Processando os dados
        for linha in linhas:
            if lendo == 0:
                if linha.startswith('Sonar Height:'):
                    dados['Sonar_gt'] = float(linha.split(':')[1].strip())
                if linha.startswith('Sonar Height with'):
                    dados['Sonar'] = float(linha.split(':')[1].strip())
                elif linha.startswith('Velocity - Linear:'):
                    partes_velocidade = linha.split(':')[1].split(',')
                    dados['Velocity Linear'] = {
                        'x': float(partes_velocidade[0].split('=')[1]),
                        'y': float(partes_velocidade[1].split('=')[1]),
                        'z': float(partes_velocidade[2].split('=')[1])
                    }
                elif linha.startswith('GPS without noise:'):
                    lendo = 1
                elif linha.startswith('Orientation quartenion:'):
                    lendo = 3
                    lidos = 0
                    dados['Orientation Quaternion'] = {
                        'X': 0.0,
                        'Y': 0.0,
                        'Z': 0.0,
                        'W': 0.0
                    }
                elif linha.startswith('Roll:'):
                    dados['Roll'] = float(linha.split(':')[1].strip())
                elif linha.startswith('Pitch:'):
                    dados['Pitch'] = float(linha.split(':')[1].strip())
                elif linha.startswith('Yaw:'):
                    dados['Yaw'] = float(linha.split(':')[1].strip())
                elif "Odometry Pose" in linha:
                    dados['Odometry Pose'] = {
                        'X': 0.0,
                        'Y': 0.0,
                        'Z': 0.0
                    }
                    lendo = 4
                    lidos = 0
            elif lendo == 1:
                partes_gps = linha.split(',')
                #dados['GPS'] = {
                #        'Altitude': float(partes_gps[0].split()[1]),
                #        'Latitude': float(partes_gps[1].split()[1]),
                #        'Longitude': float(partes_gps[2].split()[1])
                #    }
                lendo = 0
            elif lendo == 2:
                partes_gps = linha.split(',')
                #dados['GPS_gt'] = {
                #        'Altitude': float(partes_gps[0].split()[1]),
                #        'Latitude': float(partes_gps[1].split()[1]),
                #        'Longitude': float(partes_gps[2].split()[1])
                #    }
                lendo = 0
            elif lendo == 3:
                if lidos == 0:
                    dados['Orientation Quaternion']['X'] = float(linha.split(':')[1])
                elif lidos == 1:
                    dados['Orientation Quaternion']['Y'] = float(linha.split(':')[1])
                elif lidos == 2:
                    dados['Orientation Quaternion']['Z'] = float(linha.split(':')[1])
                elif lidos == 3:
                    dados['Orientation Quaternion']['W'] = float(linha.split(':')[1]) 
                    lendo = 0
                lidos += 1  
            elif lendo == 4:
                if lidos == 0:
                    dados['Odometry Pose']['X'] = float(linha.split(':')[1])
                elif lidos == 1:
                    dados['Odometry Pose']['Y'] = float(linha.split(':')[1])
                elif lidos == 2:
                    dados['Odometry Pose']['Z'] = float(linha.split(':')[1])
                    lendo = 0
                lidos += 1 

    return dados

def ler_arquivos_txt_em_pasta(pasta):
    dados_arquivos = {}
    # Verifica se o caminho da pasta existe
    if os.path.exists(pasta):
        # Itera sobre todos os arquivos na pasta
        for arquivo in os.listdir(pasta):
            # Verifica se é um arquivo .txt
            if arquivo.endswith(".txt"):
                # Constrói o caminho completo do arquivo
                caminho_completo = os.path.join(pasta, arquivo)
                # Processa o arquivo e armazena os dados em um dicionário
                dados_arquivos[arquivo] = processar_arquivo_txt(caminho_completo)
    else:
        print(f"A pasta '{pasta}' não existe.")

    return dados_arquivos

def processar_arquivo_camera(caminho_arquivo):
    dados = {}
    with open(caminho_arquivo, 'r') as f:
        linhas = f.readlines()

        for linha in linhas:
            if ':' in linha:
                chave, valor = linha.split(':', 1)
                #print (chave, valor)
                if 'height' == chave.strip():
                    dados[chave.strip()] = int(valor.strip())
                elif 'width' == chave.strip():
                    dados[chave.strip()] = int(valor.strip())
                elif 'D' == chave.strip():
                    dados[chave.strip()] = [float(i) for i in valor.split('[')[1].split(']')[0].split(',')]
                elif 'P' == chave.strip():
                    dados[chave.strip()] = [float(i) for i in valor.split('[')[1].split(']')[0].split(',')]
                elif 'K' == chave.strip():
                    dados[chave.strip()] = [float(i) for i in valor.split('[')[1].split(']')[0].split(',')]
                elif 'R' == chave.strip():
                    dados[chave.strip()] = [float(i) for i in valor.split('[')[1].split(']')[0].split(',')]
                elif 'distortion_model' == chave.strip():
                    dados[chave.strip()] = valor.strip()[1:-1]                               
                elif 'roi' == chave.strip():
                    return dados


    return dados

def processar_arquivo_geometria(caminho_arquivo):
    dados = {}
    # Processar o arquivo de geometria (se necessário)
    return dados

def processar_arquivo_textura(caminho_arquivo):
    dados = {}
    # Processar o arquivo de textura (se necessário)
    return dados

def processar_arquivo_config_robo(caminho_arquivo):
    dados = {}
    with open(caminho_arquivo, 'r') as f:
        linhas = f.readlines()
        for linha in linhas:
            if ':' in linha:
                chave, valor = linha.split(':', 1)
                dados[chave.strip()] = float(valor.strip())

    return dados

def processar_arquivo_translacao_mundo(caminho_arquivo):
    dados = {}
    with open(caminho_arquivo, 'r') as f:
        linhas = f.readlines()
        for linha in linhas:
            if ':' in linha:
                chave, valor = linha.split(':', 1)
                dados[chave.strip()] = float(valor.strip())
    
    return dados

def ler_coordenadas(caminho):
    path_bull = "/catkin_ws/src/maps_file/mesh_files/map_0/Models/"
    coordenadas = []
    models = []
    orientacao = []
    with open(caminho, 'r') as file:
            for line in file:
                coords = line.strip().split()
                if len(coords) != 8: 
                    continue

                # Converte as coordenadas para float
                x, y, z = float(coords[0].strip(',')), float(coords[1].strip(',')), float(coords[2].strip(','))
        
                wq , xq, yq, zq = float(coords[3].strip(',')), float(coords[4].strip(',')), float(coords[5].strip(',')), float(coords[6].strip(','))

                # Modelo como string
                model = coords[7].strip(',')

                coordenadas.append((x,y,z))
                models.append(path_bull + model)
                orientacao.append((wq, xq, yq, zq))

    return coordenadas, models, orientacao

# Função para converter coordenadas 3D para 2D na imagem
def world_to_screen(point, projection, view, SCR_WIDTH, SCR_HEIGHT):
    point_vec4 = glm.vec4(point[0], point[1], point[2], 1.0)
    clip_space = projection * view * point_vec4
    if clip_space.w == 0:
        return None  # Evita divisão por zero

    ndc = glm.vec3(clip_space.x / clip_space.w, clip_space.y / clip_space.w, clip_space.z / clip_space.w)

    # Transformação para coordenadas de tela
    screen_x = int((ndc.x * 0.5 + 0.5) * SCR_WIDTH)
    screen_y = int((1 - (ndc.y * 0.5 + 0.5)) * SCR_HEIGHT)  # Inverte Y pois a origem é no topo
    
    if 0 <= screen_x < SCR_WIDTH and 0 <= screen_y < SCR_HEIGHT:
        return screen_x, screen_y
    return None  # Retorna None se estiver fora da tela

def convert_bbox_inverse(xmin, ymin, xmax, ymax):
    x_center = (xmin + xmax) / 2
    y_center = (ymin + ymax) / 2
    width = xmax - xmin
    height = ymax - ymin
    return x_center, y_center, width, height
    
# process all input: query GLFW whether relevant keys are pressed/released this frame and react accordingly
# ---------------------------------------------------------------------------------------------------------
def processInput(window: GLFWwindow) -> None:
    if(glfwGetKey(window, GLFW_KEY_ESCAPE) == GLFW_PRESS):
        glfwSetWindowShouldClose(window, True)

# glfw: whenever the window size changed (by OS or user resize) this callback function executes
# ---------------------------------------------------------------------------------------------
def framebuffer_size_callback(window: GLFWwindow, width: int, height: int) -> None:
    # make sure the viewport matches the new window dimensions; note that width and 
    # height will be significantly larger than specified on retina displays.
    glViewport(0, 0, width, height)


if __name__ == "__main__":
    if len(sys.argv) != 9:
        print("Uso: python script.py <caminho_da_pasta> <arquivo_camera> <arquivo_geometria_mundo> <coordenadas_bull> <arquivo_textura> <arquivo_config_robo> <arquivo_translacao_mundo> <caminho_da_pasta_saida>")
        sys.exit(1)

    pasta = sys.argv[1]
    arquivo_camera = sys.argv[2]
    arquivo_geometria = sys.argv[3]
    arquivo_coordenadas_bull = sys.argv[4]
    arquivo_textura = sys.argv[5]
    arquivo_config_robo = sys.argv[6]
    arquivo_config_translacao_mundo = sys.argv[7]
    out_path = sys.argv[8]

    dados_arquivos = ler_arquivos_txt_em_pasta(pasta)
    dados_camera = processar_arquivo_camera(arquivo_camera)
    dados_robo = processar_arquivo_config_robo(arquivo_config_robo)
    dados_translacao_mundo = processar_arquivo_translacao_mundo(arquivo_config_translacao_mundo)

    coordenadas, models_bull, orientacao_bull = ler_coordenadas(arquivo_coordenadas_bull)
    
    # Importar objeto COLLADA
    cena = Collada(arquivo_geometria)

    # Recuperar dados da geometria do mundo
    for geometry in cena.geometries:
        for primitive in geometry.primitives:
            vertices = np.array(primitive.vertex)            
            texcoords = np.reshape(np.array(primitive.texcoordset),(-1,2))            
            normals = np.array(primitive.normal)            
            faces = np.array(primitive.vertex_index)

    
    # vertices_bull, normals_bull, indices_bull = [], [], []
    
    # for geometry_bull in bull_cena.geometries:
    #     for primitive_bull in geometry_bull.primitives:
    #         # Certifique-se de verificar se é um TriangleSet
    #         if isinstance(primitive_bull, triangleset.TriangleSet):
    #             vertices_bull.extend(primitive_bull.vertex.flatten())
    #             normals_bull.extend(primitive_bull.normal.flatten())
    #             indices_bull.extend(primitive_bull.vertex_index.flatten())
                    
    # vertices_bull = np.array(vertices_bull, dtype=np.float32)
    # normals_bull = np.array(normals_bull, dtype=np.float32)
    # indices_bull = np.array(indices_bull, dtype=np.uint32)

    # Cria um dicionario para armazenar os dados dos modelos
    models_dic = {}

    # Iterar sobre os modelos
    for model in models_bull:
        bull_cena = Collada(model)
        vertices_bull, normals_bull, indices_bull = [], [], []
        
        for geometry_bull in bull_cena.geometries:  # Pegando a geometria do modelo atual
            for primitive_bull in geometry_bull.primitives:
                if isinstance(primitive_bull, triangleset.TriangleSet):
                    vertices_bull.extend(primitive_bull.vertex.flatten())
                    normals_bull.extend(primitive_bull.normal.flatten())
                    indices_bull.extend(primitive_bull.vertex_index.flatten())

        # Converter listas para numpy arrays
        vertices_bull = np.array(vertices_bull, dtype=np.float32)
        normals_bull = np.array(normals_bull, dtype=np.float32)
        indices_bull = np.array(indices_bull, dtype=np.uint32)

        # Armazena no dicionario associando o modelo aos seus dados
        models_dic[model] = {
            "vertices": vertices_bull,
            "normals": normals_bull,
            "indices": indices_bull
        }

    mapa = []
       
    for i in range(vertices.shape[0]):
        mapa.append(vertices[i,0])
        mapa.append(vertices[i,1])
        mapa.append(vertices[i,2])
        mapa.append(normals[i,0])
        mapa.append(normals[i,1])
        mapa.append(normals[i,2])
        mapa.append(texcoords[i,0])
        mapa.append(texcoords[i,1])

    SCR_WIDTH = dados_camera['width']
    SCR_HEIGHT = dados_camera['height']

    CAMERA_MIN = 0.1
    CAMERA_MAX = 1000.0
    
    # glfw: initialize and configure
    # ------------------------------
    # glfw: initialize and configure
    # ------------------------------
    glfwInit()
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3)
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3)
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE)

    if (platform.system() == "Darwin"): # APPLE
        glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE)

    # glfw window creation
    # --------------------
    window = glfwCreateWindow(SCR_WIDTH, SCR_HEIGHT, "LearnOpenGL", None, None)
    if (window == None):

        print("Failed to create GLFW window")
        glfwTerminate()
        exit()

    glfwMakeContextCurrent(window)
    glfwSwapInterval(1)
    glfwSetFramebufferSizeCallback(window, framebuffer_size_callback)

    glEnable(GL_DEPTH_TEST)

    my_geometry = np.array(mapa, dtype=np.float32)
    indices = np.array(faces.flatten(),dtype=np.uint32)
    
    VAO = glGenVertexArrays(1)
    VBO = glGenBuffers(1)
    EBO = glGenBuffers(1)
    
    # Criar sempre arrays, mesmo se houver apenas um modelo
    VAO_BULL = np.atleast_1d(glGenVertexArrays(len(models_dic)))
    VBO_BULL = np.atleast_1d(glGenBuffers(len(models_dic)))
    EBO_BULL = np.atleast_1d(glGenBuffers(len(models_dic)))
    NBO_BULL = np.atleast_1d(glGenBuffers(len(models_dic)))

    # Bind mapa
    glBindVertexArray(VAO)

    glBindBuffer(GL_ARRAY_BUFFER, VBO)
    glBufferData(GL_ARRAY_BUFFER, my_geometry.nbytes, my_geometry.data, GL_STATIC_DRAW)

    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, EBO)
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, indices.nbytes, indices.data, GL_STATIC_DRAW)

    # position attribute
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 8 * glm.sizeof(glm.float32), None)
    glEnableVertexAttribArray(0)
    # color attribute
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 8 * glm.sizeof(glm.float32), ctypes.c_void_p(3 * glm.sizeof(glm.float32)))
    glEnableVertexAttribArray(1)
    # texture coord attribute
    glVertexAttribPointer(2, 2, GL_FLOAT, GL_FALSE, 8 * glm.sizeof(glm.float32), ctypes.c_void_p(6 * glm.sizeof(glm.float32)))
    glEnableVertexAttribArray(2)

    indices_count = []
    model_ids = list(models_dic.keys())  # Lista de modelos para indexação

    # Dicionário para mapear os modelos a seus índices
    model_index_map = {model: i for i, model in enumerate(model_ids)}

    # Iterar sobre os modelos no dicionário
    for i, model in enumerate(model_ids):
        data = models_dic[model]
        vertices_bull = data["vertices"]
        normals_bull = data["normals"]
        indices_bull = data["indices"]

        # Bind do VAO
        glBindVertexArray(VAO_BULL[i])

        # Bind e buffer de vértices
        glBindBuffer(GL_ARRAY_BUFFER, VBO_BULL[i])
        glBufferData(GL_ARRAY_BUFFER, vertices_bull.nbytes, vertices_bull, GL_STATIC_DRAW)
        #glBufferSubData(GL_ARRAY_BUFFER, 0, vertices_bull.nbytes, vertices_bull)
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * vertices_bull.itemsize, ctypes.c_void_p(0))
        glEnableVertexAttribArray(0)

        # Bind e buffer de normais
        glBindBuffer(GL_ARRAY_BUFFER, NBO_BULL[i])
        glBufferData(GL_ARRAY_BUFFER, normals_bull.nbytes, normals_bull, GL_STATIC_DRAW)
        glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 3 * normals_bull.itemsize, ctypes.c_void_p(0))
        glEnableVertexAttribArray(1)

        # Bind e buffer de índices
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, EBO_BULL[i])
        glBufferData(GL_ELEMENT_ARRAY_BUFFER, indices_bull.nbytes, indices_bull, GL_STATIC_DRAW)

        # Contagem de índices
        indices_count.append(len(indices_bull))
    
    glBindVertexArray(VAO)
    
    # load and create a texture 
    # -------------------------
    texture = glGenTextures(1)
    glBindTexture(GL_TEXTURE_2D, texture) # all upcoming GL_TEXTURE_2D operations now have effect on this texture object
    # set the texture wrapping parameters
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT)	# set texture wrapping to GL_REPEAT (default wrapping method)
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT)
    # set texture filtering parameters
    # glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR)
    # glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR)

    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST)
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST)
    
    # load image, create texture and generate mipmaps
    try:
        img = Image.open(arquivo_textura)
        #img = np.flipud(img)
        #import pdb
        #pdb.set_trace()
        out = img.transpose(Image.FLIP_TOP_BOTTOM)
    
        #out.save('/home/lucas-alves/transpose-output.png')
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, out.width, out.height, 0, GL_RGB, GL_UNSIGNED_BYTE, out.tobytes())
        glGenerateMipmap(GL_TEXTURE_2D)

        img.close()

    except:
        print("Failed to load texture")

    #Gerando a matriz da câmera
    projection = glm.perspective(calculate_fov_y(dados_camera['K'][4],dados_camera['height']), dados_camera['width']/dados_camera['height'], CAMERA_MIN, CAMERA_MAX)
 
    #print (projection)
    if dados_camera['distortion_model'] == 'plumb_bob':
        # Distortion correction
        K_zero = True
        for d in dados_camera['D']:
            if d != 0.0:
                K_zero = False
            if not K_zero:
                print ("Modelo de distorção não implementado")
                exit(-1)

    else:
        print ("Modelo de distorção não conhecido")
        exit(-1)
          
    # render loop with texture
    #-------------------------------------------------------------------------------------------------------------
    # build and compile our shader zprogram
    # ------------------------------------
    ourShader = Shader("texture.vs", "texture.fs") 
    
    
    # -----------
    for key in sorted(dados_arquivos.keys()):
        if glfwWindowShouldClose(window):
            break

        # input
        # -----
        processInput(window)

        #print (key)

        # render
        # ------
        #glClearColor(1.0, 1.0, 1.0, 1.0)
        glClearColor(0.0, 0.0, 0.0, 1.0)
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)

        # bind Texture
        glBindTexture(GL_TEXTURE_2D, texture)

        # create transformations       
        # get matrix's uniform location and set matrix
        ourShader.use()
        projectionLoc = glGetUniformLocation(ourShader.ID, "projection")
        glUniformMatrix4fv(projectionLoc, 1, GL_FALSE, glm.value_ptr(projection))

        # Rotação do robô
        # Crie matrizes de rotação individuais para cada eixo
        rotation_x = glm.rotate(glm.mat4(1.0), dados_arquivos[key]['Roll'], glm.vec3(1, 0, 0))
        rotation_y = glm.rotate(glm.mat4(1.0), dados_arquivos[key]['Pitch'], glm.vec3(0, 1, 0))
        rotation_z = glm.rotate(glm.mat4(1.0), dados_arquivos[key]['Yaw'], glm.vec3(0, 0, 1))

        robo_rotation_matrix = rotation_z * rotation_y * rotation_x

        posi_robo = glm.vec3(dados_arquivos[key]['Odometry Pose']['X'],dados_arquivos[key]['Odometry Pose']['Y'], dados_arquivos[key]['Odometry Pose']['Z'])
        camera_posi = robo_rotation_matrix*glm.vec4(dados_robo['x'],dados_robo['y'],dados_robo['z'],1.0)
        camera_posi = glm.vec3(camera_posi.x/camera_posi.w,camera_posi.y/camera_posi.w,camera_posi.z/camera_posi.w) + posi_robo

        # Rotação do câmera
        # Crie matrizes de rotação individuais para cada eixo
        rotation_x = glm.rotate(glm.mat4(1.0), dados_robo['r'], glm.vec3(1, 0, 0))
        rotation_y = glm.rotate(glm.mat4(1.0), dados_robo['p'], glm.vec3(0, 1, 0))
        rotation_z = glm.rotate(glm.mat4(1.0), dados_robo['y'], glm.vec3(0, 0, 1))
       
        camera_rotation_matrix = rotation_z * rotation_y * rotation_x

        robo_direction = glm.vec4(1.0, 0.0, 0.0,1.0)  #ros orientation
        camera_direction = robo_rotation_matrix*camera_rotation_matrix*robo_direction
        camera_direction = glm.vec3(camera_direction.x/camera_direction.w,camera_direction.y/camera_direction.w,camera_direction.z/camera_direction.w)

        up = robo_rotation_matrix*camera_rotation_matrix*glm.vec4(0.0, 0.0, 1.0,1.0)
        up = glm.vec3(up.x/up.w,up.y/up.w,up.z/up.w)
       
        # Matriz View World
        view = glm.lookAt(camera_posi, camera_posi + camera_direction , up)

        viewLoc = glGetUniformLocation(ourShader.ID, "view")
        glUniformMatrix4fv(viewLoc, 1, GL_FALSE, glm.value_ptr(view))

        # Crie matrizes de transformaçãpo para o mundo
        rotation_x_world = glm.rotate(glm.mat4(1.0), dados_translacao_mundo['roll_world'], glm.vec3(1, 0, 0))
        rotation_y_world = glm.rotate(glm.mat4(1.0), dados_translacao_mundo['pitch_world'], glm.vec3(0, 1, 0))
        rotation_z_world = glm.rotate(glm.mat4(1.0), dados_translacao_mundo['yaw_world'], glm.vec3(0, 0, 1))
        scale_world = glm.scale(glm.mat4(1.0), glm.vec3(dados_translacao_mundo['scale_x_world'], dados_translacao_mundo['scale_y_world'], dados_translacao_mundo['scale_z_world']))
        translate_world = glm.translate(glm.mat4(1.0), glm.vec3(dados_translacao_mundo['translate_x_world'], dados_translacao_mundo['translate_y_world'], dados_translacao_mundo['translate_z_world']))        

        model_transformation =  translate_world*scale_world*rotation_z_world*rotation_y_world*rotation_x_world

        model_transformationLoc = glGetUniformLocation(ourShader.ID, "model_transformation")
        glUniformMatrix4fv(model_transformationLoc, 1, GL_FALSE, glm.value_ptr(model_transformation))

        map_color = (1.0, 1.0, 1.0)
        color_loc = glGetUniformLocation(ourShader.ID, "bullColor")
        glUniform3f(color_loc, *map_color)

        # render container        
        glBindVertexArray(VAO)
        glDrawElements(GL_TRIANGLES,indices.shape[0] , GL_UNSIGNED_INT, None)

        key_f = key[:-4]

        # Renderizar os bois
        for i in range(len(coordenadas)):
            models = models_bull[i]

            # Garantir que o modelo está no mapeamento antes de tentar renderizar
            if models not in model_index_map:
                print(f"Erro: Modelo {models} não encontrado em model_index_map!")
                continue

            model_index = model_index_map[models]

            # Criar matriz de deslocamento
            translate_bull = glm.translate(glm.mat4(1.0), glm.vec3(coordenadas[i][0], coordenadas[i][1], coordenadas[i][2]))

            # Obtenha o quaternion da lista de orientações
            orientacao_quat = glm.quat(*orientacao_bull[i])

            # Converter o quaternion em uma matriz de rotação usando glm
            rotation_matrix = glm.mat4_cast(orientacao_quat)

            # Matriz composta para o boi
            bull_model_transformation = model_transformation * translate_bull * rotation_matrix

            # Enviar a matriz composta para o shader
            glUniformMatrix4fv(model_transformationLoc, 1, GL_FALSE, glm.value_ptr(bull_model_transformation))

            # Detectar o tipo do modelo pelo nome do arquivo
            model_name = os.path.basename(models).lower()

            if model_name.startswith("bull") or model_name.startswith("cow"):
                obj_color = (0.0, 0.0, 1.0)  # azul
                obj_id = 51
            elif model_name.startswith("tractor"):
                obj_color = (1.0, 1.0, 0.0)  # amarelo
                obj_id = 54
            elif model_name.startswith("pig"):
                obj_color = (1.0, 0.4, 0.7)  # rosa
                obj_id = 50
            elif model_name.startswith("horse"):
                obj_color = (0.5, 0.0, 0.5)  # roxo
                obj_id = 52
            elif model_name.startswith("deer"):
                obj_color = (0.4, 0.26, 0.13)  # marrom
                obj_id = 53
            else:
                obj_color = (1.0, 1.0, 1.0)  # fallback
                obj_id = -1  # ou algum valor inválido

            # Localização da uniform no shader
            color_loc = glGetUniformLocation(ourShader.ID, "bullColor")
            glUniform3f(color_loc, *obj_color)

            # Transformar vértices do boi para coordenadas de tela
            vertices_bull = models_dic[models]["vertices"].reshape(-1, 3)  # Reshape para Nx3

            # Obter limites do modelo
            min_x, min_y, min_z = vertices_bull.min(axis=0)
            max_x, max_y, max_z = vertices_bull.max(axis=0)

            # Gerar os 8 vértices do cubo
            aabb_corners = [
                (min_x, min_y, min_z),
                (min_x, min_y, max_z),
                (min_x, max_y, min_z),
                (min_x, max_y, max_z),
                (max_x, min_y, min_z),
                (max_x, min_y, max_z),
                (max_x, max_y, min_z),
                (max_x, max_y, max_z)
            ]

            # Verificar se algum dos vértices do cubo está visível
            algum_visivel = False
            for corner in aabb_corners:
                screen_pos = world_to_screen(bull_model_transformation * glm.vec4(*corner, 1.0),
                                            projection, view, SCR_WIDTH, SCR_HEIGHT)
                if screen_pos:
                    algum_visivel = True
                    break  # um só basta

            if not algum_visivel:
                # Nenhum canto do AABB está visível → pular boi
                continue

            # 4. Prosseguir com verificação completa dos vértices
            screen_coords = []
            for v in vertices_bull:
                screen_pos = world_to_screen(bull_model_transformation * glm.vec4(v[0], v[1], v[2], 1.0),
                                            projection, view, SCR_WIDTH, SCR_HEIGHT)
                if screen_pos:
                    screen_coords.append(screen_pos)

            if not screen_coords:  
                #print(f"[INFO] Boi {i} não aparece na imagem {key_f}, arquivo vazio será salvo.")
                bbox_data = ""  # Arquivo ficará vazio se o boi não estiver visível
            else:
                # Encontrar os limites do bounding box
                x_coords, y_coords = zip(*screen_coords)
                min_x, max_x = min(x_coords), max(x_coords)
                min_y, max_y = min(y_coords), max(y_coords)

                x_center, y_center, width_, height_ = convert_bbox_inverse(min_x, min_y, max_x, max_y)

                bbox_data = f"{obj_id} {x_center} {y_center} {width_} {height_}"
                #print(f"[SUCESSO] Boi {i} visível em {key_f}: Bounding Box: {bbox_data}")
            
            bbox_file_path = f"/catkin_ws/src/maps_file/mesh_files/map_0/bounding_box/{key_f}.txt"
            # Abrir o arquivo em modo de adição ("a" em vez de "w"), para não sobrescrever os dados anteriores
            with open(bbox_file_path, "a") as bbox_file:  # Usa "a" (append) para adicionar ao arquivo
                if bbox_data:
                    bbox_file.write(bbox_data + "\n")  # Adiciona uma nova linha para cada boi
                    print(f"[SUCESSO] Bounding Box salvo para {obj_id} {i}: {bbox_data}")
                else:
                    print(f"[INFO] {obj_id} {i} não aparece em {key_f}, nada será adicionado ao arquivo.")
                        
            # Renderizar o boi correto
            glBindVertexArray(VAO_BULL[model_index])
            glDrawElements(GL_TRIANGLES, indices_count[model_index], GL_UNSIGNED_INT, None)

        # Leitura dos pixels do framebuffer
        pixels = glReadPixels(0, 0, SCR_WIDTH, SCR_HEIGHT, GL_RGB, GL_UNSIGNED_BYTE)
        image_data = np.frombuffer(pixels, dtype=np.uint8).reshape(SCR_HEIGHT, SCR_WIDTH, 3).copy()

        # Inversão da imagem para que a parte superior seja no topo
        image_data = np.flipud(image_data)

        # Substituir as cores vivas dos objetos 3D pelos IDs exatos que o DeepLabV3 espera:
        # Bull & Cow: Azul (0, 0, 255) -> Classe 6 (6, 6, 6)
        #MUDANCA
        mask_bull = (image_data[:, :, 0] == 0) & (image_data[:, :, 1] == 0) & (image_data[:, :, 2] == 255)
        image_data[mask_bull] = [6, 6, 6]

        # Tractor: Amarelo (255, 255, 0) -> Classe 7 (7, 7, 7)
        mask_tractor = (image_data[:, :, 0] == 255) & (image_data[:, :, 1] == 255) & (image_data[:, :, 2] == 0)
        image_data[mask_tractor] = [7, 7, 7]

        # Pig: Rosa (1.0, 0.4, 0.7) -> (255, 102, 178 ou 179) -> Classe 8 (8, 8, 8)
        mask_pig = (image_data[:, :, 0] == 255) & (image_data[:, :, 1] == 102) & (np.isin(image_data[:, :, 2], [178, 179]))
        image_data[mask_pig] = [8, 8, 8]

        # Horse: Roxo (0.5, 0.0, 0.5) -> (127 ou 128, 0, 127 ou 128) -> Classe 9 (9, 9, 9)
        mask_horse = (np.isin(image_data[:, :, 0], [127, 128])) & (image_data[:, :, 1] == 0) & (np.isin(image_data[:, :, 2], [127, 128]))
        image_data[mask_horse] = [9, 9, 9]

        # Deer: Marrom (0.4, 0.26, 0.13) -> (102, 66, 33) -> Classe 10 (10, 10, 10)
        mask_deer = (image_data[:, :, 0] == 102) & (image_data[:, :, 1] == 66) & (image_data[:, :, 2] == 33)
        image_data[mask_deer] = [10, 10, 10]

        # Criar a imagem em formato RGB (3 canais, como o OpenCV/PyTorch espera) e salvar em PNG sem compressão
        image = Image.fromarray(image_data, 'RGB')
        image.save(out_path + "/" + key_f + ".png", format='PNG', compress_level=0)

        # glfw: swap buffers and poll IO events (keys pressed/released, mouse moved etc.)
        # -------------------------------------------------------------------------------
        glfwSwapBuffers(window)
        glfwPollEvents()

    #End of render loop with texture
    

    # render loop depth map
    # -----------

    # build and compile our shader program
    # ------------------------------------
    ourShader = Shader("depth.vs", "depth.fs") 
    
    for key in sorted(dados_arquivos.keys()):
        if glfwWindowShouldClose(window):
            break

        # input
        # -----
        processInput(window)

        #print (key)

        # render
        # ------
        glClearColor(0.2, 0.3, 0.3, 1.0)
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)

        # bind Texture
        glBindTexture(GL_TEXTURE_2D, 0)
        glBindTexture(GL_TEXTURE_2D, texture)

        # create transformations       
        # get matrix's uniform location and set matrix
        ourShader.use()
        projectionLoc = glGetUniformLocation(ourShader.ID, "projection")
        glUniformMatrix4fv(projectionLoc, 1, GL_FALSE, glm.value_ptr(projection))

        # Rotação do robô
        # Crie matrizes de rotação individuais para cada eixo
        rotation_x = glm.rotate(glm.mat4(1.0), dados_arquivos[key]['Roll'], glm.vec3(1, 0, 0))
        rotation_y = glm.rotate(glm.mat4(1.0), dados_arquivos[key]['Pitch'], glm.vec3(0, 1, 0))
        rotation_z = glm.rotate(glm.mat4(1.0), dados_arquivos[key]['Yaw'], glm.vec3(0, 0, 1))

        robo_rotation_matrix = rotation_z * rotation_y * rotation_x

        posi_robo = glm.vec3(dados_arquivos[key]['Odometry Pose']['X'],dados_arquivos[key]['Odometry Pose']['Y'], dados_arquivos[key]['Odometry Pose']['Z'])
        camera_posi = robo_rotation_matrix*glm.vec4(dados_robo['x'],dados_robo['y'],dados_robo['z'],1.0)
        camera_posi = glm.vec3(camera_posi.x/camera_posi.w,camera_posi.y/camera_posi.w,camera_posi.z/camera_posi.w) + posi_robo


        # Rotação do câmera
        # Crie matrizes de rotação individuais para cada eixo
        rotation_x = glm.rotate(glm.mat4(1.0), dados_robo['r'], glm.vec3(1, 0, 0))
        rotation_y = glm.rotate(glm.mat4(1.0), dados_robo['p'], glm.vec3(0, 1, 0))
        rotation_z = glm.rotate(glm.mat4(1.0), dados_robo['y'], glm.vec3(0, 0, 1))
       
        camera_rotation_matrix = rotation_z * rotation_y * rotation_x

        robo_direction = glm.vec4(1.0, 0.0, 0.0,1.0)  #ros orientation
        camera_direction = robo_rotation_matrix*camera_rotation_matrix*robo_direction
        camera_direction = glm.vec3(camera_direction.x/camera_direction.w,camera_direction.y/camera_direction.w,camera_direction.z/camera_direction.w)

        up = robo_rotation_matrix*camera_rotation_matrix*glm.vec4(0.0, 0.0, 1.0,1.0)
        up = glm.vec3(up.x/up.w,up.y/up.w,up.z/up.w)
       

        view = glm.lookAt(camera_posi, camera_posi + camera_direction , up)

        viewLoc = glGetUniformLocation(ourShader.ID, "view")
        glUniformMatrix4fv(viewLoc, 1, GL_FALSE, glm.value_ptr(view))

        # Crie matrizes de transformaçãpo para o mundo
        rotation_x_world = glm.rotate(glm.mat4(1.0), dados_translacao_mundo['roll_world'], glm.vec3(1, 0, 0))
        rotation_y_world = glm.rotate(glm.mat4(1.0), dados_translacao_mundo['pitch_world'], glm.vec3(0, 1, 0))
        rotation_z_world = glm.rotate(glm.mat4(1.0), dados_translacao_mundo['yaw_world'], glm.vec3(0, 0, 1))
        scale_world = glm.scale(glm.mat4(1.0), glm.vec3(dados_translacao_mundo['scale_x_world'], dados_translacao_mundo['scale_y_world'], dados_translacao_mundo['scale_z_world']))
        translate_world = glm.translate(glm.mat4(1.0), glm.vec3(dados_translacao_mundo['translate_x_world'], dados_translacao_mundo['translate_y_world'], dados_translacao_mundo['translate_z_world']))        

        model_transformation =  translate_world*scale_world*rotation_z_world*rotation_y_world*rotation_x_world

        model_transformationLoc = glGetUniformLocation(ourShader.ID, "model_transformation")
        glUniformMatrix4fv(model_transformationLoc, 1, GL_FALSE, glm.value_ptr(model_transformation))

        # render container        
        glBindVertexArray(VAO)
        glDrawElements(GL_TRIANGLES,indices.shape[0] , GL_UNSIGNED_INT, None)

        # Ler a profundidade como floats
        depth_pixels = glReadPixels(0, 0, SCR_WIDTH, SCR_HEIGHT, GL_DEPTH_COMPONENT, GL_FLOAT)
        depth_data = np.frombuffer(depth_pixels, dtype=np.float32).reshape(SCR_HEIGHT, SCR_WIDTH)
        depth_data = np.flipud(depth_data)

        # Salvar os dados originais como binário compactado NumPy
        z_ndc = 2 * depth_data - 1.0
        A = projection[2][2]
        B = projection[3][2]
        z_eye = B / (A + z_ndc)
        #print (np.min(z_eye))
        #print (np.max(z_eye))

        np.savez_compressed(out_path + "/../depth-segmentation-npz/" + key + "_depth_data.npz", depth_data=z_eye)


        # Normalizar os valores de profundidade para a faixa [0, 1]
        
        depth_data_normalized = (depth_data - np.min(depth_data)) / (np.max(depth_data) - np.min(depth_data))


        # Aplicar o colormap 'coolwarm' para colorir a profundidade
        colored_depth = plt.cm.coolwarm(depth_data_normalized)

        # Remover o canal alfa
        colored_depth = (colored_depth[:, :, :3] * 255).astype(np.uint8)

        # Converter para imagem PIL
        image = Image.fromarray(colored_depth)
        image.convert("RGB").save(out_path + "/../depth-segmentation/" + key + "_depth.png")

        # glfw: swap buffers and poll IO events (keys pressed/released, mouse moved etc.)
        # -------------------------------------------------------------------------------
        glfwSwapBuffers(window)
        glfwPollEvents()







    # optional: de-allocate all resources once they've outlived their purpose:
    # ------------------------------------------------------------------------
    glDeleteVertexArrays(1, (VAO,))
    glDeleteBuffers(1, (VBO,))
    glDeleteBuffers(1, (EBO,))

    # glfw: terminate, clearing all previously allocated GLFW resources.
    # ------------------------------------------------------------------
    glfwTerminate()