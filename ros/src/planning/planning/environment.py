import uuid
import copy
import re

class Environment:
    def __init__(self, topology, constraint: str, max_modules: int):
        self.topology = topology
        self.constraint = constraint
        self.max_modules = max_modules

    def get_legal_actions(self):
        return self.topology.list_valid_connections()

    def is_game_over(self):
        return self.game_result() or len(self.topology.robots) > self.max_modules

    def game_result(self):
        return self.topology.validate_against_constraint(self.constraint)

    def move(self, action):
        new_topology = copy.deepcopy(self.topology)
        new_robot_id = uuid.uuid4()
        new_topology.add_connection(
            new_robot_id,
            action.new_robot_face,
            action.host_robot_id,
            action.host_face,
        )
        return Environment(new_topology, self.constraint, self.max_modules)

    # move but in-place (no copying)
    def apply_move(self, action):
        sim_id = uuid.uuid4() 
        
        self.topology.add_connection(
            sim_id,
            action.new_robot_face,
            action.host_robot_id,
            action.host_face
        )

    # How close to satisfying constraaint?
    def calculate_heuristic(self):
        S = self.topology.get_observation()
        
        clauses = self.constraint.split(" and ")
        scores = []
        
        for clause in clauses:
            clause = clause.strip()
            
            # Regex to capture: (attribute) (operator) (value)
            # Matches "S.width >= 10" or "S.is_aligned"
            match = re.search(r"S\.(\w+)\s*(>=|<=|==)\s*(\d+)", clause)
            
            if match:
                attr, op_str, target = match.groups()
                current_val = getattr(S, attr)
                target_val = float(target)
                scores.append(self._score_numeric(current_val, op_str, target_val))
            
            # terrible way to handle boolean flags
            elif "is_" in clause: 
                is_negated = "not " in clause
                attr = re.search(r"S\.(\w+)", clause).group(1)
                current_val = getattr(S, attr)
                target = not is_negated
                scores.append(1.0 if current_val == target else 0.0)

        # average score of all clauses
        return sum(scores) / len(scores) if scores else 0.0

    def _score_numeric(self, current, op, target):
        scale = self.max_modules
        
        if op == "==":
            dist = abs(current - target)
            return max(0.0, 1.0 - (dist / scale))
        
        elif op == ">=":
            if current >= target: return 1.0
            dist = target - current
            return max(0.0, 1.0 - (dist / scale))
            
        elif op == "<=":
            if current <= target: return 1.0
            dist = current - target
            return max(0.0, 1.0 - (dist / scale))
            
        return 0.0

    def __repr__(self):
        return f"State(Value: {self.topology.get_observation()}, Constraint: {self.constraint})"
