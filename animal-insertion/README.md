# Animal Coordinates Generation

This module is responsible for generating animal positions and orientations on 3D terrain data.

The meshes can be found here:

## How to Configure Animal Distribution

To specify the animals and their spatial distribution, you must edit the file: `src/arguments.txt`.

### The file follows the format:

```
mesh-model-name\
number-of-herds\
animals-per-herd\
distribution-coefficient (sigma)

(blank line)

mesh-model-name\
number-of-herds\
...
```

- The **distribution coefficient (sigma)** controls how spread out the animals are.  
  A **higher sigma** results in **more dispersed animals**.


## Requirements

### Python Dependencies

Ensure the following Python libraries are installed:

- `laspy`
- `open3d`
- `collada`
- `rasterio`
- `math`
- `scipy.linalg`
- `Pillow`

## LAStools
You must have the las2las tool from LAStools installed and available in your system path.
Download: https://lastools.github.io/

## Media Assets
3D models and their corresponding textures should be placed in:

```animal-insertion/media/```

Make sure the .dae (Collada) files and textures are correctly referenced.

## How to Run
Execute the script with the following command:

``` 
python animal-insertion.py /tif_image /las_file /semantic_texture path_out

```

### Example

``` 
python animal-insertion.py ros-gazebo-simulation/src/maps_file/images_tif_files/map_0.tif ros-gazebo-simulation/src/maps_file/las_files/map_0.las ros-gazebo-simulation/src/maps_file/mesh_files/map_0/textura_semantic.png ros-gazebo-simulation/src/maps_file/mesh_files/map_0

```