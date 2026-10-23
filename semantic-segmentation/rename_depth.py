import os
from pathlib import Path
import shutil

def rename_files(input_dir, output_dir, prefix):
    input_dir = Path(input_dir)
    output_dir = Path(output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)

    for filename in os.listdir(input_dir):
        new_name = filename.replace('simulation', 'image').replace('.txt', '')
        new_name = f"{prefix}_{new_name}"

        old_file = input_dir / filename
        new_file = output_dir / new_name

        shutil.copy2(old_file, new_file)  # copia mantendo metadados
        print(f"Copiado e renomeado: {old_file} -> {new_file}")

# Exemplo de uso
rename_files(
    input_dir='/home/lucas-alves/Desktop/New_Dataset_LARS/mapa_17/depth_normalized_colored',
    # output_dir='/home/lucas-alves/Desktop/Depth_colored/val/images/',
    output_dir='/home/lucas-alves/Desktop/Depth_colored/train/images/',
    prefix='mapa_17'
 )

rename_files(
    input_dir='/home/lucas-alves/Desktop/New_Dataset_LARS/mapa_17/semantic-segmentation',
    # output_dir='/home/lucas-alves/Desktop/Depth_colored/val/labels/',
    output_dir='/home/lucas-alves/Desktop/Depth_colored/train/labels/',
    prefix='mapa_17'
)
