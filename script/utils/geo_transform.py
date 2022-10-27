import math
import numpy as np

def euler_from_quaternion(orientation):
    """
    Convert a quaternion into euler angles (roll, pitch, yaw)
    roll is rotation around x in radians (counterclockwise)
    pitch is rotation around y in radians (counterclockwise)
    yaw is rotation around z in radians (counterclockwise)
    """
    x,y,z,w = orientation.x_val,orientation.y_val,orientation.z_val,orientation.w_val
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll_x = math.atan2(t0, t1)

    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch_y = math.asin(t2)

    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = math.atan2(t3, t4)

    return roll_x, pitch_y, yaw_z  # in radians

def euler_to_quaternion(yaw, pitch, roll):
    qx = np.sin(roll / 2) * np.cos(pitch / 2) * np.cos(yaw / 2) - np.cos(roll / 2) * np.sin(pitch / 2) * np.sin(
        yaw / 2)
    qy = np.cos(roll / 2) * np.sin(pitch / 2) * np.cos(yaw / 2) + np.sin(roll / 2) * np.cos(pitch / 2) * np.sin(
        yaw / 2)
    qz = np.cos(roll / 2) * np.cos(pitch / 2) * np.sin(yaw / 2) - np.sin(roll / 2) * np.sin(pitch / 2) * np.cos(
        yaw / 2)
    qw = np.cos(roll / 2) * np.cos(pitch / 2) * np.cos(yaw / 2) + np.sin(roll / 2) * np.sin(pitch / 2) * np.sin(
        yaw / 2)

    return [qx, qy, qz, qw]

def rotateMultirotor(client,dyaw=0.0):
    # init multirotor pose
    pose = client.simGetVehiclePose()
    roll, pitch, yaw = euler_from_quaternion(pose.orientation)
    print('initial pose_quaternion:{}'.format(pose))
    print('initial pose_euler:{} {} {}'.format(roll, pitch, yaw))
    yaw += dyaw
    qx, qy, qz, qw = euler_to_quaternion(yaw, pitch, roll)
    pose.orientation.w_val = qw
    pose.orientation.z_val = qz
    pose.orientation.y_val = qy
    pose.orientation.x_val = qx
    client.simSetVehiclePose(pose, ignore_collison=True)
    print('changed pose_quaternion:{}'.format(pose))
    print('changed pose_euler:{} {} {}'.format(roll, pitch, yaw))