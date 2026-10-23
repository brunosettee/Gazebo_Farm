# Large-Scale Map Generation

This repository contains a Python script to generate 3D maps by combining Point Clouds and orthophotos from various areas provided by the [Swisstopo website](https://www.swisstopo.admin.ch/en/).

## Data Sources

The data sources used to generate the 3D maps are:

- **Orthophotomosaic**: [Orthophotomosaic Swissimage 10](https://www.swisstopo.admin.ch/en/orthoimage-swissimage-10)
- **Point Cloud**: [SwissSURFACE3D](https://www.swisstopo.admin.ch/en/height-model-swisssurface3d)

## How to Run the Script

You can run the script using the following command lines:

```bash
python gera_mapa.py /tif_images_file /las_point_clouds_file /path_out
```

```bash
python gera_mapa.py /tif_image_file /las_point_cloud_file /path_out x0 y0 size
```
