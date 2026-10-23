# Animal-Targeted Waypoint Generation

This module provides target-driven waypoint generation specifically designed for aerial photography of animals in farm environments.

## Features

- **Two strategies**: Naive (sequential) and Optimized (TSP-based)
- **Smart distance optimization**: Reduces total flight distance by up to 63%
- **Animal type detection**: Supports pig, bull, deer, cow variants
- **Flexible height control**: Configurable drone altitude above ground
- **Fast execution**: Optimized strategy runs in milliseconds

## Usage

### Basic Command
```bash
python3 generate_animal_waypoints.py coordinates.txt --strategy optimized --desired_height 10 --output_dir ./output
```

### With Plotting
```bash
# Show plot interactively
python3 generate_animal_waypoints.py coordinates.txt --strategy optimized --desired_height 10 --plot --output_dir ./output

# Save plot to PNG file
python3 generate_animal_waypoints.py coordinates.txt --strategy optimized --desired_height 10 --save_plot --output_dir ./output

# Both show and save
python3 generate_animal_waypoints.py coordinates.txt --strategy optimized --desired_height 10 --plot --save_plot --output_dir ./output
```

### Docker Services
```bash
# Optimized strategy (recommended)
docker-compose up animal_waypoint_generator

# Naive strategy (for comparison)
docker-compose up animal_waypoint_generator_naive
```

## Strategies

### Naive Strategy
- Visits animals in file order
- Fast generation (~0.000s)
- Higher total distance (6850m for 125 animals)
- Simple implementation

### Optimized Strategy  
- Uses Traveling Salesman Problem (TSP) optimization
- Nearest neighbor heuristic for speed
- Significantly reduced distance (2521m for 125 animals)
- 63% improvement over naive approach

## Performance Comparison

| Strategy | Distance (m) | Avg per waypoint (m) | Generation time |
|----------|--------------|----------------------|-----------------|
| Naive    | 6849.88      | 55.24               | ~0.000s         |
| Optimized| 2521.39      | 20.33               | ~0.002s         |

## File Formats

### Input: coordinates.txt
```
x, y, z, qx, qy, qz, qw, animal_type.dae
-206.68, -397.75, 166.71, 0.64, -0.09, 0.05, 0.76, pig.dae
```

### Output Files
- **Waypoints**: `animal_waypoints_[strategy].txt`
```
# x,y,z
-206.683,-397.749,176.705
-210.583,-400.149,176.725
```

- **Plots**: `animal_waypoints_[strategy]_plot.png` (if `--save_plot` used)
  - High-resolution visualization (300 DPI)
  - Color-coded animals by type
  - Flight path with start/end markers

## Animal Detection
Automatically detects and counts:
- Pigs (pig.dae)
- Bulls (bull_1.dae)  
- Deer (deer.dae)
- Cows (cow_1.dae, cow_2.dae)

## Research Applications
Ideal for:
- Precision livestock monitoring
- Animal behavior studies
- Farm management automation
- Aerial photography optimization
- UAV path planning research
