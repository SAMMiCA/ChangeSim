#!/usr/bin/env python
import airsim
import sys
import random
from utils.geo_transform import euler_from_quaternion, euler_to_quaternion
import argparse
import os

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description="follow trajectory")
    parser.add_argument("--waypoint",
                        type=str,
                        default="a")
    parser.add_argument("--velocity",
                        type=float,
                        default=0.3)
    parser.add_argument("--initial_z",
                        type=float,
                        default=0.2)
    parser.add_argument("--inverse",
                        action='store_true')
    parser.add_argument("--drive_type",
                        type=str,
                        default="forward",
                        help='forward or max_dof')
    parser.add_argument("--add_noise",
                        action='store_true')
    parser.add_argument("--direction",
                        type=float,
                        default='0.0')
    args = parser.parse_args()

    client = airsim.MultirotorClient()
    client.confirmConnection()
    client.enableApiControl(True)
    print("arming the drone...")
    client.armDisarm(True)
    drive_type = airsim.DrivetrainType.ForwardOnly if args.drive_type == 'forward' \
        else airsim.DrivetrainType.MaxDegreeOfFreedom
    # take off multirotor
    state = client.getMultirotorState()
    if state.landed_state == airsim.LandedState.Landed:
        print("taking off...")
        client.takeoffAsync(timeout_sec=1).join()
        client.moveToZAsync(z=-args.initial_z, velocity=args.velocity) # minus is upper way in z-axis
    else:
        client.hoverAsync().join()
    state = client.getMultirotorState()
    if state.landed_state == airsim.LandedState.Landed:
        print("take off failed...")
        sys.exit(1)

    path1 = []
    path2 = []
    for i in range(0,200):
        obj_name = "waypoint_{}_{}".format(args.waypoint,i)
        print(obj_name)
        position1 = client.simGetObjectPose(obj_name).position
        position2 = client.simGetObjectPose(obj_name).position

        if not position1.x_val != position1.x_val: # verify that val is not NaN
            # position1.x_val += -2.0
            path1.append(position1)
            # position2.x_val += -2.0
            position2.x_val += random.gauss(0, 0.2)
            position2.y_val += random.gauss(0, 0.2)
            position2.z_val += random.gauss(0, 0.2)
            path2.append(position2)
        print(position1)
        print('\n')

    if args.add_noise: path1 = path2
    if args.inverse: path1 = path1[::-1]

    input('when you are ready to move, press any key\n')
    print("start 'move_on_path1' ")
    try:
        client.moveOnPathAsync(path1, args.velocity, 3e+38,
                               drive_type, airsim.YawMode(False, args.direction), -1, 1).join()
    except KeyboardInterrupt:
        client.landAsync().join()
        print("disarming...")
        client.armDisarm(False)
        client.enableApiControl(False)
        print("done.")

    os.system("pkill -9 python")