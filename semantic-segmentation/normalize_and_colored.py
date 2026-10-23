import numpy as np
from pathlib import Path
import matplotlib.pyplot as plt
import matplotlib.cm as cm
from PIL import Image

# Diretórios
input_dir = Path("/home/lucas-alves/Desktop/New_Dataset_LARS/mapa_17/depth-segmentation-npz/")
output_dir = Path("/home/lucas-alves/Desktop/New_Dataset_LARS/mapa_17/depth_normalized_colored/")
output_dir.mkdir(parents=True, exist_ok=True)

for file in input_dir.glob("*.npz"):
    data = np.load(file)
    
    if "depth_data" not in data:
        print(f"[AVISO] 'depth_data' não encontrado em {file.name}. Pulando.")
        continue

    d = data["depth_data"].astype(np.float32)

    # Evita log(0)
    d = np.clip(d, 1e-6, None)

    # Normalização
    log_d = np.log(d)
    u = np.mean(log_d)
    norm_d = log_d - u

    # Reescala para [0, 1] para aplicar colormap
    norm_d_min = np.min(norm_d)
    norm_d_max = np.max(norm_d)
    norm_d_scaled = (norm_d - norm_d_min) / (norm_d_max - norm_d_min + 1e-8)

    # Aplica colormap inferno
    cmap = cm.get_cmap('inferno')
    colored_img = cmap(norm_d_scaled)[:, :, :3]  # descarta canal alfa

    # Converte para uint8
    colored_img_uint8 = (colored_img * 255).astype(np.uint8)
    img_pil = Image.fromarray(colored_img_uint8)

    # Salva como PNG
    output_path = output_dir / (file.stem + ".png")
    img_pil.save(output_path)
    print(f"[OK] Salvo: {output_path}")
