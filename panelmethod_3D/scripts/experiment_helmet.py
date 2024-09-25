#!/usr/bin/env python3

import sys
import time
import meshio
import numpy as np
import pandas as pd
import PGFlow3D
import pygalmesh
from djitellopy import TelloSwarm
from voliere import Vehicle as Target
from voliere import VolierePosition
import glob


class Logger:
    """Basic logger class to keep data and write to CSV file"""

    def __init__(self):
        self.data = {}

    def add_drone(self, drone_id):
        """Add new drone to the logger

        Args:
            drone_id (str): drone number as key
        """
        self.data[drone_id] = []

    def log(self, drone_id, data):
        """Log data for the drone
        Order is Time - Position - Velocity - Quat - Control - Target
        """
        self.data[drone_id].append(data)

    def save(self, filename):
        """Save data to CSV file

        Args:
            filename (str): desired filename{+drone_id.csv} to save the data
        """
        print("Inside Log.Save")
        for key, item in self.data.items():
            df = pd.DataFrame(
                item,
                columns=["Time", "Position", "Velocity", "Quat", "Control", "Target"],
            )
            df.to_csv(filename + str(key) + ".csv", index=False)


def create_ac_list(drones):
    """
    Create list of list for drones
    Order : OpTR - AcId -- IP
    """
    ac_list = []
    for drone in drones:
        ac_list.append([drone, drone, f"192.168.1.{drone}"])
    return ac_list


MAX_VEL = 1.4
MIN_VEL = 0.4


# def velocity_func(distance):
#     if distance >= 0.5:
#         return MAX_VEL
#     slope = (MAX_VEL - MIN_VEL) / 2
#     return MIN_VEL + distance * slope


# def velocity_func(distance):
#     distance -= 0.10
#     if distance >= 0.5:
#         return MAX_VEL
#     if distance < 0.10:
#         return 0.2
#     return MIN_VEL + (MAX_VEL - MIN_VEL) / 0.5 / 0.5 * distance * distance


def velocity_func(distance):
    return 0.6


def main():
    pgflow = PGFlow3D.Case()

    FREQ = 50
    DRONES = ["60", "61", "62"]
#    DRONES =  ["60"]

    log = Logger()
    for drone in DRONES:
        log.add_drone(drone)

    ac_list = create_ac_list(DRONES)

    # Extract Optitrack and and AC IDs
    id_list = [ac[1] for ac in ac_list]
    ip_list = [ac[2] for ac in ac_list]

    # Create TelloSwarm
    swarm = TelloSwarm.fromIps(ip_list)
    # Assign AC IDs
    for i, idx in enumerate(id_list):
        swarm.tellos[i].set_ac_id(idx)

    print("Connecting to Tello Swarm...")
    swarm.connect()
    print("Connected to Tello Swarm...")

    ac_id_list = [[ac[0], ac[1]] for ac in ac_list]

    id_dict = dict((id[0], id[1]) for id in ac_id_list)  # RigidBodyID, AircraftID

    vehicles = {}
    # Set Vehicles
    for vehicle in swarm.tellos:
        vehicles[vehicle.ac_id] = vehicle

    # Add Helmet Here
    id_dict["888"] = "888"
    vehicles["888"] = Target("888")

    voliere = VolierePosition(id_dict, vehicles, freq=100, vel_samples=6)
    print("Calling Voliere Run")
    voliere.run()
    time.sleep(2)

    # Add drones to PGFLOWc
    for _ in DRONES:
        pgflow.addVehicle()
        pgflow.addGoal()

    pgflow.setGoalReachDistance(0.15)

    for i, mesh_file in enumerate(glob.glob("*.obj")):
        # print(f"Adding mesh {i} {mesh_file}")
        pgflow.addBuilding()
        pgflow.initBuildingFromOBJFile(i, mesh_file)
        pgflow.setBuildingSafetyVelocity(i, 0.0)

    # Create locations
    n_vehicles = pgflow.getNumberOfVehicles()

    # pgflow.setGoalPosition(0, np.array([-4, +1.5, 1.25]))
    # pgflow.setGoalPosition(1, np.array([-4, -1.5, 1.5]))
    # pgflow.setGoalPosition(2, np.array([0, -3.5, 1.5]))

    # pgflow.setVehiclePriority(0, 2.0)
    # pgflow.setVehiclePriority(1, 1.0)

    locs = [[] for _ in range(n_vehicles)]

    # Simulation starts
    sim_start_time = time.time()

    validtrack = 0
    validtrackmax = FREQ * 3

    try:
        swarm.takeoff()

        starttime = time.time()
#        while time.time() - sim_start_time < 180:
        while ((time.time() - sim_start_time < 120) and (validtrack < validtrackmax)):

            if (vehicles["888"].valid): validtrack = 0
            else: validtrack = validtrack + 1

            if time.time() - starttime > 1.5:
                start = time.time()
                print(f"--- {start-starttime} ---")
                helmet_pos = vehicles["888"].position

                for i, vehicle in enumerate(swarm.tellos):
                    pgflow.setVehiclePosition(i, np.array(vehicle.get_position_enu()))
                    locs[i].append(pgflow.getVehiclePosition(i))
                    pgflow.setGoalPosition(i, helmet_pos)
                    # print(f"Helmet position {helmet_pos}")
                # pgflow.calculateVehicleVelocities(2.0)
                # pgflow.calculateVehicleVelocitiesWithGuidanceStream(0.05,0.5)
                pgflow.calculateVehicleVelocitiesWithGuidanceStreamAdaptively(
                    0.05, 10.5, 10.5, 1.00
                )
                for i, vehicle in enumerate(swarm.tellos):
                    vel = pgflow.getVehicleVelocity(i)
                    pos = vehicle.get_position_enu()
                    if pos[2] < 0.4:
                        vel[2] = max(0.0, vel[2])
                    elif pos[2] > 3.0:
                        vel[2] = min(0.0, vel[2])
                    vel_mag = np.linalg.norm(vel)
                    min_dist = pgflow.getGoalDistance(i)
                    desired_mag = velocity_func(min_dist)
                    vel_norm = (vel / vel_mag) * desired_mag
                    if vel_mag < 0.00001:
                        vel_norm = np.array([0, 0, 0])
                    vehicle.send_velocity_enu(vel_norm, vehicle.heading)
                    log.log(
                        vehicle.ac_id,
                        [
                            time.time() - starttime,
                            vehicle.get_position_enu(),
                            vehicle.get_velocity_enu(),
                            vehicle.get_quaternion(),
                            vel_norm,
                            pgflow.getGoalPosition(i),
                        ],
                    )

                time.sleep(max(1.0 / FREQ - (time.time() - start), 0))

        #### Save the simulation results ###########################
        log.save("FlightTest_")
        try:
            swarm.move_down(int(40))
            swarm.land()
        except:
            pass
        voliere.stop()
        swarm.end()

        # Write the vehicle locations to a VTK file
        cells = [
            (
                "vertex",
                np.array([[i] for i in range(len(locs[0]))]),
            )
        ]

        for i in range(n_vehicles):
            mesh = meshio.Mesh(locs[i], cells)
            mesh.write(f"vehicle_{i}.vtk")

    except (KeyboardInterrupt, SystemExit):
        print("Shutting down natnet interfaces...")
        try:
            swarm.move_down(int(40))
            swarm.land()
        except:
            pass
        voliere.stop()
        swarm.end()
        # Write the vehicle locations to a VTK file
        cells = [
            (
                "vertex",
                np.array([[i] for i in range(len(locs[0]))]),
            )
        ]

        for i in range(n_vehicles):
            mesh = meshio.Mesh(locs[i], cells)
            mesh.write(f"vehicle_{i}.vtk")
        log.save("FlightTest_")
        time.sleep(1)

    except OSError:
        print("Natnet connection error")
        swarm.move_down(int(40))
        swarm.land()
        voliere.stop()
        swarm.end()
        sys.exit(-1)


if __name__ == "__main__":
    main()
