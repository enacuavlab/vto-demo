#!/usr/bin/env python3

import meshio
import numpy as np
import copy


#for i in [2,4,5,6,7]:
for i in [6]:
    fname = f"new_building_{i}_buff.obj"
    mesh = meshio.read(fname)
    triangles = copy.deepcopy(mesh.cells_dict["triangle"])
    triangles = np.array(triangles)
    # triangles = np.roll(triangles, 1, axis=1)
    new_triangles = copy.deepcopy(triangles)
    new_triangles[:, 0] = triangles[:, 2]
    new_triangles[:, 2] = triangles[:, 0]
    new_mesh = meshio.Mesh(points=mesh.points, cells=[("triangle", new_triangles)])
    meshio.write(fname, new_mesh)
