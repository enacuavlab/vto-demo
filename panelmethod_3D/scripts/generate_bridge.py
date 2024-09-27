#!/usr/bin/env python3
import pygalmesh

p = pygalmesh.Polygon2D([[0, 1], [0, -1], [0.75, -1], [0.75, 1]])
edge_size = 0.1
domain = pygalmesh.Extrude(
        p,
        [0.0, 0.0, 0.5],
        0,
        edge_size
        )
mesh = pygalmesh.generate_surface_mesh(
        domain,
        min_facet_angle=1.0,
        max_radius_surface_delaunay_ball=0.15,
        max_facet_distance=0.009,
        verbose=False
        )
mesh.write("bridge.obj")
