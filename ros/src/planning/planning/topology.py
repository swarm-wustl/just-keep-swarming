import numpy as np
from dataclasses import dataclass
import networkx as nx
from typing import Tuple
from collections import Counter

Cell = Tuple[int, int]

class Face:
    FRONT = "front"
    BACK = "back"
    LEFT = "left"
    RIGHT = "right"

class Orientation:
    NORTH = "north"
    SOUTH = "south"
    WEST = "west"
    EAST = "east"


@dataclass
class Robot:
    orientation: Orientation
    position: Cell

@dataclass
class OpenPort:
    host_robot_id: str
    host_face: str  # The face on the host robot available for connection
    new_robot_face: str  # The face on the new robot available for connection
    target_position: Cell

def rotate_direction(direction: Cell, orientation: Orientation) -> Cell:

    dx, dy = direction
    
    match orientation:
        case Orientation.NORTH:
            return (dx, dy)
            
        case Orientation.SOUTH:
            return (-dx, -dy)
            
        case Orientation.WEST:
            return (-dy, dx)
            
        case Orientation.EAST:
            return (dy, -dx)
    
       
def add_cells(
    cell1: Cell,
    cell2: Cell,
) -> Cell:
    return (cell1[0] + cell2[0], cell1[1] + cell2[1])


to_angle = {
    "north": 0, "front": 0,
    "west": 90, "left": 90,
    "south": 180, "back": 180,
    "east": 270, "right": 270
}

to_orient = {
    0: Orientation.NORTH,
    90: Orientation.WEST,
    180: Orientation.SOUTH,
    270: Orientation.EAST
}


def face2direction(face: Face):
    match face:
        case Face.FRONT:
            return (0, 1)
        case Face.BACK:
            return (0, -1)
        case Face.LEFT:
            return (-1, 0)
        case Face.RIGHT:
            return (1, 0)


@dataclass
class TopologyObservation:
    size: int # the number of modules
    width: int # the number of modules between the rightmost and leftmost module
    length: int # the number of modules between the topmost and bottommost module
    diameter: int # the nx.diameter of the topology
    max_degree: int # the highest number of connections of each module
    endpoints: int # the number of modules with only one connection
    radius: int # the manhattan distance of the furthest module from the origin
    is_aligned: bool # are all modules facing the same way?
    endpoints_aligned: bool # are all endpoint (degree 1) modules facing the same way?
    head_orientation: str # the orientation of the robot furthest from origin
    is_symmetric_x: bool # is symmetric horizontally?
    is_symmetric_y: bool # is symmetric vertically?

import ast
import operator

# Supported operators map
OPS = {
    ast.Eq: operator.eq, ast.NotEq: operator.ne,
    ast.Lt: operator.lt, ast.LtE: operator.le,
    ast.Gt: operator.gt, ast.GtE: operator.ge,
    ast.And: lambda x, y: x and y,
    ast.Or: lambda x, y: x or y,
}

def safe_eval(expr: str, context: dict) -> bool:
    """
    Safely evaluates a boolean expression involving context variables.
    Only allows: Attribute access (S.width), Comparisons (<, >), and Boolean logic (and, or).
    """
    try:
        tree = ast.parse(expr, mode='eval')
    except SyntaxError:
        return False

    def _eval(node):
        # Base Literals (Numbers, True/False)
        if isinstance(node, ast.Constant):
            return node.value
            
        # Variable Names (S)
        elif isinstance(node, ast.Name):
            return context.get(node.id)
            
        # Attribute Access (S.width)
        elif isinstance(node, ast.Attribute):
            obj = _eval(node.value)
            return getattr(obj, node.attr)
            
        # Comparisons (S.width > 5)
        elif isinstance(node, ast.Compare):
            left = _eval(node.left)
            for op, right_node in zip(node.ops, node.comparators):
                right = _eval(right_node)
                if not OPS[type(op)](left, right):
                    return False
            return True
            
        # Boolean Logic (A and B)
        elif isinstance(node, ast.BoolOp):
            values = [_eval(v) for v in node.values]
            # Reduce the list using the operator (AND/OR)
            if isinstance(node.op, ast.And):
                return all(values)
            elif isinstance(node.op, ast.Or):
                return any(values)

        raise ValueError(f"Unsafe or unsupported operation: {type(node)}")

    return _eval(tree.body)

def visualize_ascii(topology):
    if not topology.robots:
        print("Empty swarm.")
        return

    # 1. Gather data and determining bounds
    # Map (x,y) -> orientation_string
    grid_map = {r.position: r.orientation for r in topology.robots.values()}
    
    xs = [pos[0] for pos in grid_map.keys()]
    ys = [pos[1] for pos in grid_map.keys()]
    
    min_x, max_x = min(xs), max(xs)
    min_y, max_y = min(ys), max(ys)
    
    # 2. Define Character Mappings
    # Matches your Orientation class strings
    char_map = {
        "north": "^",
        "south": "v",
        "east":  ">",
        "west":  "<"
    }

    print(f"\n--- Swarm Visualization (Size: {len(topology.robots)}) ---")
    
    # 3. Render
    # We iterate Y from Max to Min so 'North' is visually 'Up'
    for y in range(max_y, min_y - 1, -1):
        row_str = ""
        for x in range(min_x, max_x + 1):
            if (x, y) in grid_map:
                orient = grid_map[(x, y)]
                # Use the arrow, fallback to '#' if orientation is missing
                symbol = char_map.get(orient, "#")
            else:
                symbol = "." # Empty space
            
            # Add an extra space for roughly square aspect ratio in terminal
            row_str += symbol + " " 
        print(row_str)
    print("----------------------------------------------\n")

class SwarmTopology:
    def __init__(self, origin_robot_id: str):
        origin_pos = (0, 0)
        self.robots = {origin_robot_id: Robot(Orientation.NORTH, origin_pos)}

        # set of occupied coordinates for O(1) lookups
        self.occupied_set = {origin_pos}

    def to_graph(self) -> nx.Graph:
        G = nx.Graph()
    
        # coordinate (x, y) -> robot_id
        pos_to_id = {robot.position: r_id for r_id, robot in self.robots.items()}
    
        for r_id, robot in self.robots.items():
            G.add_node(r_id, pos=robot.position)
    
            x, y = robot.position
    
            # We only need to check 2 directions to catch every edge exactly once 
            potential_neighbors = [
                (x + 1, y), # East
                (x, y + 1)  # North
            ]
    
            for n_pos in potential_neighbors:
                if n_pos in pos_to_id:
                    neighbor_id = pos_to_id[n_pos]
                    G.add_edge(r_id, neighbor_id)
    
        return G

    def add_connection(
        self,
        new_robot_id: str,
        new_robot_face: Face,
        current_robot_id: str,
        current_robot_face: Face,
    ) -> bool:
        current_robot_ids = self.robots.keys()
        if current_robot_id not in current_robot_ids:
            raise ValueError(f"Robot {current_robot_id} does not exist in network")
        elif new_robot_id in current_robot_ids:
            raise ValueError(f"Robot {current_robot_id} already exists in network")
    
        current_robot = self.robots[current_robot_id]
    
        raw_direction = face2direction(current_robot_face)
        oriented_direction = rotate_direction(raw_direction, current_robot.orientation)
        new_robot_pos = add_cells(current_robot.position, oriented_direction)
    
        occupied = new_robot_pos in self.occupied_set
        if occupied:
            raise ValueError(f"There is already a robot at {new_robot_pos}")
    
        current_global_angle = to_angle[current_robot.orientation] + to_angle[current_robot_face]
        target_angle = (current_global_angle + 180 - to_angle[new_robot_face]) % 360
        new_robot_orientation = to_orient[target_angle]
    
        self.robots[new_robot_id] = Robot(new_robot_orientation, new_robot_pos)
        self.occupied_set.add(new_robot_pos)
    
        return True

    def remove_robot(self, robot_id: str):
        if robot_id in self.robots:
            self.occupied_set.remove(self.robots[robot_id].position)
            del self.robots[robot_id]

    def list_valid_connections(self):
        valid_ports = []
    
        # Cache occupied positions
        occupied_positions: Set[Cell] = {r.position for r in self.robots.values()}
    
        faces = [Face.FRONT, Face.BACK, Face.LEFT, Face.RIGHT]
    
        for r_id, robot in self.robots.items():
            for face in faces:
                raw_direction = face2direction(face)
                oriented_direction = rotate_direction(raw_direction, robot.orientation)
                neighbor_pos = add_cells(robot.position, oriented_direction)
    
                # not empty -> valid connection point
                if neighbor_pos not in occupied_positions:
                    # valid_ports.append(OpenPort(r_id, face, neighbor_pos))
                    valid_ports += [OpenPort(r_id, face, new_face, neighbor_pos) for new_face in faces]
    
        return valid_ports

    def get_observation(self) -> TopologyObservation:
        G = self.to_graph()
    
        x_coords = [r.position[0] for r in self.robots.values()]
        y_coords = [r.position[1] for r in self.robots.values()]
        degrees = [d for n, d in G.degree()]
    
        orientations = [r.orientation for r in self.robots.values()]
        counts = Counter(orientations)
    
        is_aligned = (len(counts) == 1)
    
        endpoint_orientations = [self.robots[n].orientation for n, d in G.degree() if d == 1]
        endpoints_aligned = len(set(endpoint_orientations)) <= 1
    
        head_robot = max(self.robots.values(), key=lambda r: abs(r.position[0]) + abs(r.position[1]))
        head_orientation = head_robot.orientation
    
        positions = set(r.position for r in self.robots.values())
        xs = [p[0] for p in positions]
        ys = [p[1] for p in positions]
    
        min_x, max_x = min(xs), max(xs)
        min_y, max_y = min(ys), max(ys)
    
        sum_x = min_x + max_x
        sum_y = min_y + max_y
    
        # X sym: For every block at (x,y), there must be a block at (sum_x - x, y)
        is_symmetric_x = all((sum_x - x, y) in positions for x, y in positions)
    
        # Y sym: For every block at (x,y), there must be a block at (x, sum_y - y)
        is_symmetric_y = all((x, sum_y - y) in positions for x, y in positions)
    
        obs = TopologyObservation(
            size=len(self.robots),
            width=max(x_coords) - min(x_coords) + 1,
            length=max(y_coords) - min(y_coords) + 1,
            diameter=nx.diameter(G) if nx.is_connected(G) and len(G) > 0 else 0,
            max_degree=max(degrees) if degrees else 0,
            endpoints=degrees.count(1),
            radius=max(abs(x) + abs(y) for x, y in zip(x_coords, y_coords)), # Manhattan radius
            is_aligned=is_aligned,
            endpoints_aligned=endpoints_aligned,
            head_orientation=head_orientation,
            is_symmetric_x=is_symmetric_x,
            is_symmetric_y=is_symmetric_y,
        )
    
        return obs

    def validate_against_constraint(self, constraint_str: str) -> bool:
        obs = self.get_observation()
    
        try:
            return safe_eval(constraint_str, {"S": obs})
        except Exception as e:
            print(f"Constraint Error: {e}")
            return False
