import numpy as np
from typing import Tuple
from dataclasses import dataclass
import networkx as nx
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

class SwarmTopology:
    def __init__(self, origin_robot_id: str):
        self.robots = {origin_robot_id: Robot(Orientation.NORTH, (0, 0))}

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
    
        occupied = new_robot_pos in (robot.position for robot in self.robots.values())
        if occupied:
            raise ValueError(f"There is already a robot at {new_robot_pos}")
    
        current_global_angle = to_angle[current_robot.orientation] + to_angle[current_robot_face]
        target_angle = (current_global_angle + 180 - to_angle[new_robot_face]) % 360
        new_robot_orientation = to_orient[target_angle]
    
        self.robots[new_robot_id] = Robot(new_robot_orientation, new_robot_pos)

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
