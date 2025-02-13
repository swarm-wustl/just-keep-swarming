from geometry_msgs.msg import Pose, Point, Quaternion


def create_pose(point, quat):
    pose = Pose()
    pose.position = point
    pose.orientation = quat
    return pose


def create_quat(x, y, z, w):
    quat = Quaternion()
    quat.x = x
    quat.y = y
    quat.z = z
    quat.w = w
    return quat


def create_point(x, y, z):
    point = Point()
    point.x = x
    point.y = y
    point.z = z
    return point
