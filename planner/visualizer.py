import os
import time
import numpy as np
import pybullet as p

VOXEL_SIZE = 0.5
GRID_SIZE = 50
FILENAME = "/home/michael/Projects/edgeslam-path-planner/edgeslam/exported-data/occupancy-grid.txt"

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

    except ValueError as e:
        print(f"An error occurred while parsing the file: {e}")
        return []
    except Exception as e:
        print(f"An unexpected error occurred: {e}")
        return []

    return grid
    
def get_voxel_center(voxel_indices):
    return (np.array(voxel_indices) + 0.5) * VOXEL_SIZE

def place_voxels(voxels, color=(0.7, 0.2, 0.2, 1.0)):
    global voxel_grid
    global update_vis
    for v_index in voxels:
        pos = [get_voxel_center(i) for i in v_index]
        visualShapeId = p.createVisualShape(
                shapeType=p.GEOM_BOX,
                halfExtents=[VOXEL_SIZE / 2, VOXEL_SIZE / 2, VOXEL_SIZE / 2],
                rgbaColor=color,
                visualFramePosition=pos
            )
        p.createMultiBody(baseVisualShapeIndex=visualShapeId, basePosition=[0,0,0])


imported_grid = import_occupancy_grid(FILENAME, GRID_SIZE)

if imported_grid:
    print("Successfully imported the grid!")
    print("Dimensions: i={}, j={}, k={}".format(
        len(imported_grid),
        len(imported_grid[0]),
        len(imported_grid[0][0])
    ))
else:
    print("Failed to import the grid.")


#we need to load the grid in pybullet

physicsClient = p.connect(p.GUI)
p.configureDebugVisualizer(p.COV_ENABLE_GUI,0)

while p.isConnected():
    p.stepSimulation()
    time.sleep(1/120)