#!/usr/bin/env python3

import glob
import re

import numpy as np
import pygalmesh
import shapely


def create_poly_mesh(base, height, filename):
    """Create polygonal mesh from base and height"""
    # Create a 2D Polygonal Base
    p = pygalmesh.Polygon2D(base)
    # Define the height of the Extrusion
    max_edge_size_at_feature_edges = 0.1
    rotation = 0  # in radians
    # Create the Domain by Extruding the Polygonal Base
    domain = pygalmesh.Extrude(
        p,
        [0.0, 0.0, height],
        rotation,
        max_edge_size_at_feature_edges,
    )
    # # Generate the Mesh
    mesh = pygalmesh.generate_surface_mesh(
        domain,
        min_facet_angle=1.0,
        max_radius_surface_delaunay_ball=0.15,
        max_facet_distance=0.009,
        verbose=True,
    )
    # Write the Mesh to a VTK File
    mesh.write(filename)


def main():
    buffer_size = 0.15
    files = glob.glob("*.npy")

    for i, file in enumerate(files):
        points = np.load(file)
        height = None
        building_number = None
        try:
            building_number = int(re.findall(r"_(\d+)_", file).pop())
            height = float(re.findall(r"(-?\d+\.\d+).npy$", file).pop())
            print(building_number)
        except Exception:
            print(f"The file {file} is not in the correct format")
            continue
        geo = shapely.Polygon(points)
        buff = shapely.buffer(geo, buffer_size)
        create_poly_mesh(
            buff.exterior.coords[:],
            height + buffer_size,
            f"new_building_{building_number}_buff.obj",
        )


if __name__ == "__main__":
    main()
