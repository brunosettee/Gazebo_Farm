import sys
import laspy
import rasterio
from rasterio.windows import from_bounds

if len(sys.argv) != 4:
    print("Uso: python3 corta_tif.py <input_tif> <input_las> <output_tif>")
    sys.exit(1)

input_tif = sys.argv[1]
input_las = sys.argv[2]
output_tif = sys.argv[3]

# Abre o LAS
with laspy.open(input_las) as las_file:
    header = las_file.header
    minx, miny = header.x_min, header.y_min
    maxx, maxy = header.x_max, header.y_max


# Recorta o TIF usando as coordenadas do LAS
print(f"2. Recortando a imagem: {input_tif}")
with rasterio.open(input_tif) as src:
    window = from_bounds(minx, miny, maxx, maxy, src.transform)
    
    clipped_img = src.read(window=window)
    clipped_transform = src.window_transform(window)
    
    profile = src.profile.copy()
    profile.update({
        'driver': 'GTiff',
        'height': clipped_img.shape[1],
        'width': clipped_img.shape[2],
        'transform': clipped_transform
    })
    
    with rasterio.open(output_tif, 'w', **profile) as dst:
        dst.write(clipped_img)

print(f"O TIF foi salvo em: {output_tif}")