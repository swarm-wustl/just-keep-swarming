from typing import Tuple

Cell = Tuple[int, int]

class Orientation:
    NORTH = "north"
    SOUTH = "south"
    WEST = "west"
    EAST = "east"

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

import re

def extract_code_block(llm_output: str) -> str:
    pattern = r"```(?:[\w]*\n)?(.*?)```"
    
    match = re.search(pattern, llm_output, re.DOTALL)
    
    if match:
        return match.group(1).strip()
    return None

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
