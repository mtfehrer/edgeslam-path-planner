import os
import time
import numpy as np
import pybullet as p

VOXEL_SIZE = 0.5
GRID_SIZE = 50
FILENAME = "/home/michael/Projects/edgeslam-path-planner/planner/occupancy-grid.txt"
UNKNOWN_VOXEL = "0"
FREE_VOXEL = "1"
OCCUPIED_VOXEL = "2"
RED = [0.5, 0.0, 0.0, 1.0]
BLUE = [0, 0, 0.5, 1]

def import_occupancy_grid(filename, grid_size):
    if not os.path.exists(filename):
        print(f"Error: File not found at {filename}")
        return []

    grid = []
    try:
        with open(filename, 'r') as f:
            for i in range(grid_size):
                j_slice = []
                for j in range(grid_size):
                    line = f.readline()
                    if not line:
                        raise ValueError("Unexpected end of file. The grid is smaller than expected.")
                    
                    k_row = [char for char in line.strip().split(' ') if char]

                    if len(k_row) != grid_size:
                        raise ValueError(f"Data mismatch at slice i={i}, row j={j}. "
                                         f"Expected {grid_size} values, but found {len(k_row)}.")
                    
                    j_slice.append(k_row)
                
                grid.append(j_slice)

                if i < grid_size - 1:
                    blank_line = f.readline()
                    if blank_line.strip() != "":
                        raise ValueError(f"Formatting error: Expected a blank line after slice i={i}, but found content.")

    except Exception as e:
        print(f"An unexpected error occurred: {e}")
        exit()

    return grid
    
def get_voxel_center(voxel_indices):
    return [(v + 0.5) * VOXEL_SIZE for v in voxel_indices]

grid = import_occupancy_grid(FILENAME, GRID_SIZE)

physicsClient = p.connect(p.GUI)

p.configureDebugVisualizer(p.COV_ENABLE_GUI,0)

voxel_positions = []

for i in range(GRID_SIZE):
    for j in range(GRID_SIZE):
        for k in range(GRID_SIZE):
            if grid[i][j][k] == OCCUPIED_VOXEL:
                voxel_positions.append(get_voxel_center((i, j, k)))

num_voxels = len(voxel_positions)

visual_shape_id = p.createVisualShapeArray(shapeTypes=[p.GEOM_BOX] * num_voxels,
                                           visualFramePositions=voxel_positions,
                                           rgbaColors=[RED] * num_voxels)

# visual_shape_id = p.createVisualShapeArray(shapeTypes=[p.GEOM_BOX] * num_voxels,
#                                            halfExtents=[[VOXEL_SIZE / 2, VOXEL_SIZE / 2, VOXEL_SIZE / 2]] * num_voxels,
#                                            visualFramePositions=voxel_positions,
#                                            rgbaColors=[RED] * num_voxels)

p.createMultiBody(baseVisualShapeIndex=visual_shape_id)

while p.isConnected():
    p.stepSimulation()
    time.sleep(1/120)