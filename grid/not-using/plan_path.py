import math
import time
import csv

VOXEL_SIZE = 1

occupied_voxels = set()
free_voxels = set()

def get_voxel(point):
    x, y, z = point
    sx = sy = sz = float(VOXEL_SIZE)
    i = math.floor(x / sx)
    j = math.floor(y / sy)
    k = math.floor(z / sz)
    return (i, j, k)

while (True):
    points = []
    with open('points.csv', 'r') as file:
        reader = csv.reader(file)
        for row in reader:
            point = tuple(float(val) for val in row)
            points.append(point)
    occupied_voxels = set([get_voxel(p) for p in points])
    time.sleep(1)