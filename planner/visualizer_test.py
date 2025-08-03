import pyvista as pv
import numpy as np
import time

cube = pv.Cube() 

plotter = pv.Plotter()

plotter.show(interactive_update=True)

for i in range(100000):
    plotter.clear()

    grid = pv.PolyData(np.array([i, 0, 0]))
    glyphs = grid.glyph(scale=False, geom=cube)
    plotter.add_mesh(glyphs, color='red', show_edges=True)

    plotter.update()

    time.sleep(1)

plotter.close()