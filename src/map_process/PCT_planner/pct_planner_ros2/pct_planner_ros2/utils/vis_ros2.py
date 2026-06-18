from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path


def traj_to_path(traj, frame_id='map', stamp=None):
    path_msg = Path()
    path_msg.header.frame_id = frame_id
    if stamp is not None:
        path_msg.header.stamp = stamp

    for waypoint in traj:
        pose = PoseStamped()
        pose.header.frame_id = frame_id
        if stamp is not None:
            pose.header.stamp = stamp
        pose.pose.position.x = float(waypoint[0])
        pose.pose.position.y = float(waypoint[1])
        pose.pose.position.z = float(waypoint[2])
        pose.pose.orientation.w = 1.0
        path_msg.poses.append(pose)

    return path_msg
