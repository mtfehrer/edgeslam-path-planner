import os
import time
import numpy as np
import pybullet as p

VOXEL_SIZE = 0.5
GRID_SIZE = 100
FILENAME = "/home/michael/Projects/edgeslam-path-planner/planner/occupancy-grid.txt"
EMPTY_VOXEL = "0"

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

grid = import_occupancy_grid(FILENAME, GRID_SIZE)

link_positions = []
link_colors = []
base_pos = None
base_color = None

physicsClient = p.connect(p.GUI)

voxel_visual_shape = p.createVisualShape(
    shapeType=p.GEOM_BOX,
    halfExtents=[VOXEL_SIZE / 2] * 3
)

p.configureDebugVisualizer(p.COV_ENABLE_GUI,0)

for i in range(GRID_SIZE):
    for j in range(GRID_SIZE):
        for k in range(GRID_SIZE):
            if grid[i][j][k] != EMPTY_VOXEL:
                pos = get_voxel_center((i, j, k))
                color = (0, 0.5, 0, 1) if grid[i][j][k] == "1" else (0.5, 0, 0, 1)
                if base_pos is None:
                    base_pos = pos
                    base_color = color
                else:
                    link_positions.append(np.array(pos) - base_pos)
                    link_colors.append(color)

print(f"link_positions: {link_positions}")

grid_body_id = p.createMultiBody(
    baseVisualShapeIndex=voxel_visual_shape,
    basePosition=base_pos,
    linkVisualShapeIndices=[voxel_visual_shape] * len(link_positions),
    linkPositions=link_positions,
    linkOrientations=[[0, 0, 0, 1]] * len(link_positions),
    linkInertialFramePositions=[[0, 0, 0]] * len(link_positions),
    linkInertialFrameOrientations=[[0, 0, 0, 1]] * len(link_positions),
    linkParentIndices=[0] * len(link_positions),
    linkJointTypes=[p.JOINT_FIXED] * len(link_positions),
    linkJointAxis=[[0, 0, 1]] * len(link_positions)
)

p.changeVisualShape(grid_body_id, -1, rgbaColor=base_color)
for link_index, color in enumerate(link_colors):
    p.changeVisualShape(grid_body_id, link_index, rgbaColor=color)

while p.isConnected():
    p.stepSimulation()
    time.sleep(1/120)