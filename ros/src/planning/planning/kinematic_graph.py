from geometry_msgs.msg import Pose, Point, Quaternion
import numpy as np
from scipy.spatial.transform import Rotation as R
from collections import deque
from dataclasses import dataclass
from typing import List, Dict

MODULE_SIZE = 1.0
HALF_SIZE = MODULE_SIZE / 2

class Face:
    FRONT = "front"
    BACK = "back"
    LEFT = "left"
    RIGHT = "right"
    TOP = "top"
    BOTTOM = "bottom"

@dataclass
class Connection:
    target_id: int
    source_face: str
    target_face: str

class KinematicGraph:
    def __init__(self):
        self.adj: Dict[int, List[Connection]] = {}
        self.modules: set = set()

    def add_connection(self, u_id: int, u_face: str, v_id: int, v_face: str):
        if u_id not in self.adj: self.adj[u_id] = []
        if v_id not in self.adj: self.adj[v_id] = []
        self.modules.add(u_id)
        self.modules.add(v_id)
        self.adj[u_id].append(Connection(v_id, u_face, v_face))
        self.adj[v_id].append(Connection(u_id, v_face, u_face))

    def get_neighbors(self, u_id: int) -> List[Connection]:
        return self.adj.get(u_id, [])

    def list_valid_connections(self):
        # returns a list of connections that could be made
        return []

def get_face_transform(face_name: str) -> np.ndarray:
    mat = np.eye(4)
    if face_name == Face.FRONT:
        mat[:3, 3] = [HALF_SIZE, 0, 0]
    elif face_name == Face.BACK:
        mat[:3, 3] = [-HALF_SIZE, 0, 0]
        mat[:3, :3] = R.from_euler('y', 180, degrees=True).as_matrix()
    elif face_name == Face.LEFT:
        mat[:3, 3] = [0, HALF_SIZE, 0]
        mat[:3, :3] = R.from_euler('z', 90, degrees=True).as_matrix()
    elif face_name == Face.RIGHT:
        mat[:3, 3] = [0, -HALF_SIZE, 0]
        mat[:3, :3] = R.from_euler('z', -90, degrees=True).as_matrix()
    elif face_name == Face.TOP:
        mat[:3, 3] = [0, 0, HALF_SIZE]
        mat[:3, :3] = R.from_euler('y', -90, degrees=True).as_matrix()
    elif face_name == Face.BOTTOM:
        mat[:3, 3] = [0, 0, -HALF_SIZE]
        mat[:3, :3] = R.from_euler('y', 90, degrees=True).as_matrix()
    return mat

def get_flip_transform() -> np.ndarray:
    mat = np.eye(4)
    mat[:3, :3] = R.from_euler('z', 180, degrees=True).as_matrix()
    return mat

def unfold_graph(graph: KinematicGraph, root_id: int, base_pose_matrix: np.ndarray = np.eye(4)) -> Dict[int, Pose]:
    """
    Unfolds graph and returns a Dictionary of ROS2 Pose messages keyed by ID.
    """
    global_transforms = {}
    global_transforms[root_id] = base_pose_matrix
    
    queue = deque([root_id])
    visited = {root_id}
    
    # ROS2 Pose objects result
    poses = {}
    
    while queue:
        curr_id = queue.popleft()
        T_curr = global_transforms[curr_id]
        
        # Convert matrix to ROS Pose
        trans = T_curr[:3, 3]
        quat = R.from_matrix(T_curr[:3, :3]).as_quat() # x, y, z, w
        
        p = Pose()
        p.position = Point(x=trans[0], y=trans[1], z=trans[2])
        p.orientation = Quaternion(x=quat[0], y=quat[1], z=quat[2], w=quat[3])
        poses[curr_id] = p
        
        neighbors = graph.get_neighbors(curr_id)
        for conn in neighbors:
            neighbor_id = conn.target_id
            if neighbor_id in visited:
                continue
            
            T_curr_face = get_face_transform(conn.source_face)
            T_flip = get_flip_transform()
            T_neighbor_face = get_face_transform(conn.target_face)
            T_neighbor_face_inv = np.linalg.inv(T_neighbor_face)
            
            T_next = T_curr @ T_curr_face @ T_flip @ T_neighbor_face_inv
            
            global_transforms[neighbor_id] = T_next
            visited.add(neighbor_id)
            queue.append(neighbor_id)
            
    return poses

def parse_topology_text(text: str) -> KinematicGraph:
    graph = KinematicGraph()
    lines = text.strip().split('\n')
    for line in lines:
        parts = line.strip().split()
        if len(parts) != 2: continue
        
        def split_conn(s):
            p = s.split('.')
            return int(p[0]), p[1].lower()
            
        try:
            u_id, u_face = split_conn(parts[0])
            v_id, v_face = split_conn(parts[1])

            # LLM outputs 1-indexed, robots use 0-indexing
            graph.add_connection(u_id - 1, u_face, v_id - 1, v_face)
        except Exception as e:
            print(f"Skipping bad line: {line} -> {e}")
    return graph
