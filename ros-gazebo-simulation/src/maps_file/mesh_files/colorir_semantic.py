import numpy as np
import cv2
import os
import sys

OLD_COLORMAP = {
    (1, 1, 1): 0,    # 1: (135, 206, 250)
    (2, 2, 2): 1,    # 2: (0, 128, 0)
    (3, 3, 3): 2,    # 3: (107, 142, 35)
    (6, 6, 6): 3,    # 6: (255, 69, 0)
    (9, 9, 9): 4,    # 9: (139, 0, 0)
    (17, 17, 17): 5,  # 17: (245, 222, 179)
    (0, 0, 255): 6, # Azul (Bull)
    (255, 255, 0): 7, # Amarelo (Trator)
    (255, 102, 178): 8, # Rosa (Pig)
    (128, 0, 128): 9, # Roxo (Cavalo)
    (102,66,33): 10,  # Marrom (Deer)
}

NEW_COLORMAP = {
    0: (135, 206, 250),
    1: (0, 128, 0),
    2: (107, 142, 35),
    3: (255, 69, 0),
    4: (139, 0, 0),
    5: (245, 222, 179),
    6: (0, 0, 255),
    7: (255, 255, 0),
    8: (255, 102, 178),
    9: (128, 0, 128),
    10: (102, 66, 33),
}

def convert_colormap(image_path, output_path):
    img = cv2.imread(image_path)
    img_rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)

    out_img = np.zeros_like(img_rgb)

    for old_color, label in OLD_COLORMAP.items():
        new_color = NEW_COLORMAP[label]
        mask = np.all(img_rgb == old_color, axis=-1)
        out_img[mask] = new_color

    out_bgr = cv2.cvtColor(out_img, cv2.COLOR_RGB2BGR)
    cv2.imwrite(output_path, out_bgr)
    print(f"Imagem convertida salva em: {output_path}")

def process_images(imagens_pasta, output_pasta):
    for image_file in os.listdir(imagens_pasta):
        if not image_file.lower().endswith((".png", ".jpg", ".jpeg")):
            continue
        
        image_path = os.path.join(imagens_pasta, image_file)
        base_name = os.path.splitext(image_file)[0]
        output_image_path = os.path.join(output_pasta, f"{base_name}_colored.png")
        
        convert_colormap(image_path, output_image_path)

if __name__ == "__main__":
    #imagens_pasta = "/catkin_ws/src/maps_file/mesh_files/map_0/semantic-segmentation"
    #output_pasta = "/catkin_ws/src/maps_file/mesh_files/map_0/semantic-segmentation-colored"

    imagens_pasta = "map_0/semantic-segmentation"
    output_pasta = "map_0/semantic-segmentation-colored"

    os.makedirs(output_pasta, exist_ok=True)
    
    process_images(imagens_pasta, output_pasta)
