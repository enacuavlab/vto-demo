#!/usr/bin/env python3

import time

import numpy as np
from scipy.spatial import ConvexHull
from voliere import Vehicle as Target
from voliere import VolierePosition


def main():

    BUILDINGS = [
        "881",
        "882",
        "883",
        "884",
        "885",
        "886",
        "887",
        "889",
    ]  
#    BUILDINGS = ["881", "883", "885", "886", "887", "889"]  # 888 Helmet, 890 Soft Ball

    ac_id_list = [[building, building] for building in BUILDINGS]

    id_dict = dict((id[0], id[1]) for id in ac_id_list)  # RigidBodyID, AircraftID

    vehicles = {}
    # Add Buildings to track
    for building in BUILDINGS:
        vehicles[building] = Target(building)
    time.sleep(2)
    voliere = VolierePosition(id_dict, vehicles, freq=100, vel_samples=6)
    time.sleep(2)
    voliere.run()
    time.sleep(2)

    for i, building in enumerate(BUILDINGS):
        points = np.array(voliere.buildingMarkerDict[building])[:, :2].astype(float)
        hull = ConvexHull(points)
        h = np.max(np.array(voliere.buildingMarkerDict[building])[:, 2:].astype(float))
        np.save(f"hull_{i}_h_{h:6.4f}", points[hull.vertices])

    voliere.stop()


if __name__ == "__main__":
    main()
