import math
from geometry_msgs.msg import PoseStamped, Quaternion
from tf2_ros import Buffer
from tf2_geometry_msgs import do_transform_pose

def is_zero_quat(q: Quaternion) -> bool:
    return abs(q.x) < 1e-9 and abs(q.y) < 1e-9 and abs(q.z) < 1e-9 and abs(q.w) < 1e-9

def transform_pose(tf_buffer: Buffer, pose: PoseStamped, target_frame: str, timeout_sec: float = 0.5) -> PoseStamped:
    if pose.header.frame_id == target_frame:
        return pose
    tf = tf_buffer.lookup_transform(
        target_frame,
        pose.header.frame_id,
        pose.header.stamp,
        timeout_sec=timeout_sec,
    )
    out = do_transform_pose(pose, tf)
    out.header.frame_id = target_frame
    return out

def offset_along_pose_x(p: PoseStamped, dx: float) -> PoseStamped:
    """
    Move along pose's local +X axis by dx (meters).
    """
    # quaternion -> rotation matrix (only needed for X axis direction)
    q = p.pose.orientation
    # normalize (just in case)
    norm = math.sqrt(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w)
    if norm < 1e-9:
        # if no orientation, treat local x as global x
        out = PoseStamped()
        out.header = p.header
        out.pose = p.pose
        out.pose.position.x += dx
        return out
    x = q.x / norm; y = q.y / norm; z = q.z / norm; w = q.w / norm

    # local +X axis in world:
    # R * [1,0,0]
    vx = 1 - 2*(y*y + z*z)
    vy = 2*(x*y + w*z)
    vz = 2*(x*z - w*y)

    out = PoseStamped()
    out.header = p.header
    out.pose = p.pose
    out.pose.position.x += dx * vx
    out.pose.position.y += dx * vy
    out.pose.position.z += dx * vz
    return out
