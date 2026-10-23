import cv2
import sys
import os

def convert_bbox_inverse(x_center, y_center, width, height):
    xmin = int(x_center - width / 2)
    ymin = int(y_center - height / 2)
    xmax = int(x_center + width / 2)
    ymax = int(y_center + height / 2)
    return xmin, ymin, xmax, ymax

def desenhar_bounding_boxes(imagem_path, bbox_txt, output_path):
    imagem = cv2.imread(imagem_path)

    if imagem is None:
        print(f"Erro: Não foi possível carregar a imagem '{imagem_path}'")
        return

    try:
        with open(bbox_txt, 'r') as f:
            linhas = f.readlines()
        
        if not linhas:
            print(f"[INFO] O arquivo '{bbox_txt}' está vazio. Nenhum bounding box será desenhado.")
            return

        for linha in linhas:
            valores = linha.strip().split()

            # Verifica se há pelo menos 5 valores (classe + bbox)
            if len(valores) < 5:
                continue

            # Ignora a classe e usa os valores absolutos em pixels
            x_center, y_center, width, height = map(float, valores[1:5])
            min_x, min_y, max_x, max_y = convert_bbox_inverse(x_center, y_center, width, height)

            cor = (0, 255, 0)  # Verde (BGR)
            espessura = 2
            cv2.rectangle(imagem, (min_x, min_y), (max_x, max_y), cor, espessura)

            # Opcional: desenhar o ID/classe acima do bbox
            classe = valores[0]
            cv2.putText(imagem, f"ID:{classe}", (min_x, min_y - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, cor, 1)

    except Exception as e:
        print(f"Erro ao ler o arquivo '{bbox_txt}': {e}")
        return

    os.makedirs(output_path, exist_ok=True)

    nome_arquivo = os.path.basename(imagem_path)
    output_file = os.path.join(output_path, nome_arquivo)
    cv2.imwrite(output_file, imagem)
    print(f"[SUCESSO] Imagem salva em: {output_file}")

import re

def extrair_id_arquivo(nome):
    """Extrai o ID numérico comum do nome do arquivo"""
    match = re.search(r'(\d+\.\d+)', nome)
    return match.group(1) if match else None

def processar_pastas(imagens_pasta, bbox_pasta, output_pasta):
    imagens_dict = {
        extrair_id_arquivo(nome): nome
        for nome in os.listdir(imagens_pasta)
        if nome.lower().endswith(".png")
    }

    for bbox_file in os.listdir(bbox_pasta):
        if not bbox_file.endswith(".txt"):
            continue

        id_comum = extrair_id_arquivo(bbox_file)
        if id_comum is None:
            print(f"[ERRO] ID não encontrado em '{bbox_file}'")
            continue

        imagem_nome = imagens_dict.get(id_comum)
        if not imagem_nome:
            print(f"[ERRO] Imagem correspondente para '{bbox_file}' com ID {id_comum} não encontrada!")
            continue

        imagem_path = os.path.join(imagens_pasta, imagem_nome)
        bbox_path = os.path.join(bbox_pasta, bbox_file)
        desenhar_bounding_boxes(imagem_path, bbox_path, output_pasta)

# Uso: python script.py <pasta_imagens> <pasta_bounding_boxes> <pasta_saida>
if __name__ == "__main__":
    if len(sys.argv) != 4:
        print("Uso: python script.py <pasta_imagens> <pasta_bounding_boxes> <pasta_saida>")
        sys.exit(1)

    imagens_pasta = sys.argv[1]
    bbox_pasta = sys.argv[2]
    output_pasta = sys.argv[3]

    processar_pastas(imagens_pasta, bbox_pasta, output_pasta)
