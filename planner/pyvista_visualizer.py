import os
import numpy as np
import pyvista as pv

VOXEL_SIZE = 1
GRID_SIZE = 50
FILENAME = "/home/michael/Projects/edgeslam-path-planner/planner/occupancy-grid.txt"
UNKNOWN_VOXEL = "0"
FREE_VOXEL = "1"
OCCUPIED_VOXEL = "2"
RED = [0.5, 0.0, 0.0, 1.0]
BLUE = [0, 0, 0.5, 1]

def print_grid():
    for matrix in grid:
        for row in matrix:
            s = ""
            for c in row:
                s += c
            print(s)
        print("")

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
    

grid = import_occupancy_grid(FILENAME, GRID_SIZE)

free_voxel_positions = []
occupied_voxel_positions = []

for i in range(GRID_SIZE):
    for j in range(GRID_SIZE):
        for k in range(GRID_SIZE):
            if grid[i][j][k] == FREE_VOXEL:
                free_voxel_positions.append([i, j, k])
            if grid[i][j][k] == OCCUPIED_VOXEL:
                occupied_voxel_positions.append([i, j, k])

num_free_voxels = len(free_voxel_positions)
num_occupied_voxels = len(occupied_voxel_positions)

plotter = pv.Plotter()
cube = pv.Cube()

free_voxels_grid = pv.PolyData(np.array(free_voxel_positions))
occupied_voxels_grid = pv.PolyData(np.array(occupied_voxel_positions))

free_voxels_glyphs = free_voxels_grid.glyph(scale=False, geom=cube)
occupied_voxels_glyphs = occupied_voxels_grid.glyph(scale=False, geom=cube)

plotter.add_mesh(free_voxels_glyphs, color='skyblue', show_edges=True)
plotter.add_mesh(occupied_voxels_glyphs, color='skyblue', show_edges=True)

plotter.show()
