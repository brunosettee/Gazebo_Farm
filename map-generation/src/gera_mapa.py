import laspy
import open3d as o3d
import random
import os
import numpy as np
import rasterio
from rasterio.windows import Window
from rasterio.merge import merge
from collada import *
from PIL import  Image, ImageFilter

XMIN, YMIN, SIZE_AREA = 9000, 9000, 12000
                 
def gera_image(xmin, ymin, size, original_image_path, out_dir_path):
    # Criação da lista de arquivos de entrada
    input_files = [
        f"{original_image_path}map_{i}.tif"
        for i in range(9)
    ]

    output_file = f"{out_dir_path}/merged_tif.tif"

    src_files_to_mosaic = []
    for file in input_files:
        src = rasterio.open(file)
        src_files_to_mosaic.append(src)

    # Mesclagem das imagens
    mosaic, out_trans = merge(src_files_to_mosaic)

    out_meta = src.meta.copy()
    out_meta.update({
        "driver": "GTiff",
        "height": mosaic.shape[1],
        "width": mosaic.shape[2],
        "transform": out_trans
    })

    # Escrita da imagem mesclada em um arquivo temporário
    with rasterio.open(output_file, "w", **out_meta) as dest:
        dest.write(mosaic)

    # Abrindo a imagem mesclada para recorte
    with rasterio.open(output_file) as src:
 
        # Definindo a janela de recorte
        window = Window(xmin, ymin, size, size)

        # Lendo a janela recortada
        clipped_img = src.read(window=window)

        # Obtendo a transformação da janela recortada
        clipped_transform = src.window_transform(window)

        # Atualizando o perfil para salvar como JPEG
        profile = src.profile
        profile.update(driver='JPEG')
        profile.update({
            'width': size,
            'height': size,
            'transform': clipped_transform,
            'compress': 'JPEG',
            'quality': 100
        })

        # Salvando a imagem recortada como JPEG
        output_jpeg = f"{out_dir_path}/textura.jpeg"
        with rasterio.open(output_jpeg, 'w', **profile) as dst:
            dst.write(clipped_img)

    return clipped_transform

def create_collada_file(clipped_transform,original_las_path,out_dir_path, original_image_path, gerar_labels = True):

    xmin, ymin = clipped_transform * (0, 0)
    xmax, ymax = clipped_transform * (SIZE_AREA, SIZE_AREA)

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

    # print("Gerando merged_las.las")
    os.system(f"lasmerge64 -i {original_las_path}*.las -o {out_dir_path}merged_las.las")

    # print("Gerando filtered.las")
    os.system(f"las2las64 -i {out_dir_path}merged_las.las -o {out_dir_path}filtered_las.las -keep_xy {xmin_clipp} {ymin_clipp} {xmax_clipp} {ymax_clipp}")

    # #Abrir o arquivo LAS e ler os pontos
    main_las_file_path = f"{out_dir_path}/filtered_las.las"
    las_file = laspy.read(main_las_file_path)

    points = np.vstack((las_file.x, las_file.y, las_file.z)).transpose()

    if(gerar_labels):
        print("Gerando labels")
        labels = np.array(las_file.classification)

        #mapeamento de cores para os labels
        cores = {
        0: (0, 0, 0),  
        1: (1,1,1),      
        2: (2,2,2),
        3: (3,3,3),
        4: (4,4,4),
        5: (5,5,5),
        6: (6,6,6),
        7: (7,7,7),
        8: (8,8,8),
        9: (9,9,9),
        10: (10,10,10),
        11: (11,11,11),
        12: (12,12,12),
        13: (13,13,13),
        14: (14,14,14),
        15: (15,15,15),
        16: (16,16,16),
        17: (17,17,17),
        }

        # Tamanho da imagem (largura, altura)
        SIZE = (SIZE_AREA, SIZE_AREA)
        # Criar uma nova imagem com fundo branco
        imagem = Image.new('RGB', SIZE, color='white')

        from scipy.spatial import KDTree
        # Construir a árvore KD
        arvore_kd = KDTree(points[:,:2])

        print("Gerando segmentação semantica!")
        for i in range(SIZE_AREA):
            for j in range(SIZE_AREA):
                x,y = clipped_transform * (i, j)
                ponto_referencia = np.array([x,y])
                # Buscar o ponto mais próximo
                indice_mais_proximo = arvore_kd.query(ponto_referencia)[1]
                label = labels[indice_mais_proximo]
                cor = cores.get(label, (0,0,0))
                imagem.putpixel((i, j), cor)
        
        #Salvar a imagem como PNG
        imagem_filtrada = imagem.filter(ImageFilter.MedianFilter(size=5))
        imagem_filtrada.save(out_dir_path + '/textura_semantic.png', compress_level=0)
        print("Segmentação semantica Gerada!")
    
    #Filtrar os pontos dentro da area desejada
    indices = np.where((points[:, 0] >= xmin_clipp) & (points[:, 0] <= xmax_clipp) & (points[:, 1] >= ymin_clipp) & (points[:, 1] <= ymax_clipp))

    #Selecionar apenas os pontos dentro da area desejada
    points_inside_area = points[indices]

    #Subtracao dos pontos para tirar deslocamento
    x_mean = np.mean(points_inside_area[:,0])
    y_mean = np.mean(points_inside_area[:,1])
    z_mean = np.mean(points_inside_area[:,2])

    #Subtracao dos pontos para tirar deslocamento
    points_inside_area = points_inside_area - np.array([ x_mean,y_mean,z_mean])

    #Criar uma nova nuvem de pontos com os pontos dentro da area desejada
    nuvem_pontos = o3d.geometry.PointCloud()
    nuvem_pontos.points = o3d.utility.Vector3dVector(points_inside_area)

    #Visualizar e salvar a nuvem de pontos dentro da area desejada
    #o3d.visualization.draw_geometries([nuvem_pontos])

    #Estimar as normais 
    nuvem_pontos.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.2, max_nn=30))

    normals_nuvem = np.asarray(nuvem_pontos.normals)

    # Vetor para calcular o produto escalar
    vector = np.array([0, 0, 1])

    # Calcular os produtos escalares entre as normais e o vetor (0,0,1)
    dot_products = np.dot(normals_nuvem, vector)

    # Inverter o sentido das normais onde o produto escalar é negativo
    inverted_normals = normals_nuvem.copy()

    inverted_normals[dot_products < 0] *= -1

    # Atribuir as novas normais à nuvem de pontos
    nuvem_pontos.normals = o3d.utility.Vector3dVector(inverted_normals)

    #Aplicar a reconstrucao de Poisson
    poisson_mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(nuvem_pontos, depth=10, scale=1.0)[0]

    #vertices
    vert_floats = np.array(poisson_mesh.vertices)
    print(vert_floats.shape)

    #Filtrando a mesh para não ficar fora das bordas
    vertices_to_remove = []   
    for i in range(vert_floats.shape[0]):
        vertices_to_remove.append(not ((vert_floats[i, 0] >= xmin_clipp - x_mean) & (vert_floats[i, 0] <= xmax_clipp - x_mean) & (vert_floats[i, 1] >= ymin_clipp - y_mean) & (vert_floats[i, 1] <= ymax_clipp - y_mean)))
    
    poisson_mesh.remove_vertices_by_mask(vertices_to_remove)
    
    #Salvar a malha gerada em formato PLY
    #o3d.io.write_triangle_mesh(out_dir_path + "/mesh.ply", poisson_mesh)
    #vertices

    vert_floats = np.array(poisson_mesh.vertices)
    #print(vert_floats)

    #normais
    normal_floats = np.array(poisson_mesh.vertex_normals)
    #print(normal_floats)

    #indices
    indices_ = np.array(poisson_mesh.triangles)
    print(indices_.shape)

    #Fazer o Flat do array de vertices e indices
    vert_floats_flat = vert_floats.flatten()

    #Setar as normais para 0,0,1
    normal_floats = np.full_like(normal_floats, [0, 0, 1])
    #print(normal_floats)
    normal_floats_flat = normal_floats.flatten()
    #replicar os indices 3 vezes
    indices_repeat = np.repeat(indices_,3)

    xmin = xmin - x_mean
    ymin = ymin - y_mean
    xmax = xmax - x_mean
    ymax = ymax - y_mean

    #Calcular a largura e altura da malha
    largura = xmax - xmin
    altura = ymax - ymin

    u = (vert_floats[:, 0] - xmin) / largura
    v = 1- (vert_floats[:, 1] - ymin) / altura

    m1uv = np.zeros(u.shape[0]*2)
    for i in range(u.shape[0]):
        m1uv[i*2] = u[i]
        m1uv[i*2 + 1] = v[i]

    image_name = "textura.jpeg"
    mesh = Collada()
    image = material.CImage("material_0_1_0-image", image_name)
    surface = material.Surface("material_0_1_0-image-surface", image)
    sampler2d = material.Sampler2D("material_0_1_0-image-sampler", surface)
    map1 = material.Map(sampler2d, "UVSET0")

    # Aqui pode ser editado para definir outras propriedades do material 
    effect = material.Effect("effect0", [surface, sampler2d], "lambert", diffuse=map1)

    mat = material.Material("material0", "mymaterial", effect)
    mesh.images.append(image)
    mesh.effects.append(effect)
    mesh.materials.append(mat)
    vert_src = source.FloatSource("cubeverts-array", np.array(vert_floats_flat), ('X', 'Y', 'Z'))
    normal_src = source.FloatSource("cubenormals-array", np.array(normal_floats_flat), ('X', 'Y', 'Z'))
    m1uv_src = source.FloatSource("cubeuvs-array", np.array(m1uv), ('S', 'T'))
    geom = geometry.Geometry(mesh, "geometry0", "mycube", [vert_src, normal_src,m1uv_src])

    input_list = source.InputList()
    input_list.addInput(0, 'VERTEX', "#cubeverts-array")
    input_list.addInput(1, 'NORMAL', "#cubenormals-array")
    input_list.addInput(2, 'TEXCOORD', "#cubeuvs-array")


    triset = geom.createTriangleSet(indices_repeat, input_list, "materialref")
    geom.primitives.append(triset)
    mesh.geometries.append(geom)

    matnode = scene.MaterialNode("materialref", mat, inputs=[])
    geomnode = scene.GeometryNode(geom, [matnode])
    node = scene.Node("node0", children=[geomnode])

    myscene = scene.Scene("myscene", [node])
    mesh.scenes.append(myscene)
    mesh.scene = myscene

    mesh.write(out_dir_path + "/mesh.dae")
    # Abrir o arquivo em modo de escrita
    with open(out_dir_path + '/shift_values.txt', 'w') as arquivo:
        # Escrever os valores no arquivo, separados por espaço ou nova linha
        arquivo.write(f"{x_mean} {y_mean} {z_mean}\n")
        
# Exemplo de uso
if __name__ == "__main__":
    import sys

    if len(sys.argv) != 4 and  len(sys.argv) != 7 :
        print("Uso: python gera_mapa.py /tif_image /las_file path_out")
        print ("Or")
        print("Uso: python gera_mapa.py /tif_image /las_file path_out x0 y0 size")
        sys.exit(1)
    elif len(sys.argv) == 4:
        original_image_path = sys.argv[1]
        original_las_path = sys.argv[2]
        out_dir_path = sys.argv[3]
    else:
        original_image_path = sys.argv[1]
        original_las_path = sys.argv[2]
        out_dir_path =  sys.argv[3]
        XMIN = int(sys.argv[4])
        YMIN = int(sys.argv[5])
        SIZE_AREA = int(sys.argv[6])

    clipped_transform = gera_image(XMIN,YMIN,SIZE_AREA,original_image_path,out_dir_path)
    create_collada_file(clipped_transform,original_las_path,out_dir_path, original_image_path)