import random
import copy
import uuid
from topology import Face

class DFSBacktracking:
    def __init__(self, constraints: str):
        self.constraint_str = constraints
        self.found_solutions = []

    def solve(self, current_topology, max_modules=10):
        # Check if success
        if current_topology.validate_against_constraint(self.constraint_str):
            return current_topology

        # If we hit size limit, prune this branch
        if len(current_topology.robots) >= max_modules:
            return None

        # If we wanted to have a heuristic to influence which move we should make, it would be here
        possible_moves = current_topology.list_valid_connections()
        random.shuffle(possible_moves) 

        for move in possible_moves:
            new_topo = copy.deepcopy(current_topology)
            new_id = uuid.uuid4()
            new_topo.add_connection(new_id, move.host_face, move.host_robot_id, move.host_face)

            # Could potentially early-prune here
            result = self.solve(new_topo, max_modules)
            
            if result:
                return result # success

        return None # dead end


class IDDFSSolver:
    def __init__(self, constraint_str: str):
        self.constraint_str = constraint_str

    def solve(self, seed_topology, max_modules=20):
        print(f"Goal: {self.constraint_str}")
        
        # Iterative Deepening Loop
        # We start looking for a solution of size 1, then size 2...
        for depth_limit in range(1, max_modules + 1):
            print(f"Searching at max depth: {depth_limit}...")
            
            # Run DFS with this strict limit
            result = self._dfs(seed_topology, depth_limit)
            
            if result:
                return result
        
        return None

    def _dfs(self, topology, depth_limit):
        # 1. Check Constraint (Success)
        if topology.validate_against_constraint(self.constraint_str):
            # We must return a COPY, because the recursion will unwind and 
            # delete the robots from the 'topology' object otherwise.
            return copy.deepcopy(topology)

        # 2. Check Depth Limit (Base Case)
        # We use the current number of robots as the 'depth'
        if len(topology.robots) >= depth_limit:
            return None

        # 3. Get Moves
        # Convert to list so we can shuffle (randomness helps avoid loops)
        valid_moves = list(topology.list_valid_connections())
        random.shuffle(valid_moves)

        for move in valid_moves:
            # Try all faces
            for new_face in [Face.FRONT, Face.BACK, Face.LEFT, Face.RIGHT]:
                new_id = str(uuid.uuid4())[:8]
                
                # A. DO (Mutate in place)
                try:
                    success = topology.add_connection(
                        new_id, new_face, move.host_robot_id, move.host_face
                    )
                except ValueError:
                    success = False

                if success:
                    # B. RECURSE
                    result = self._dfs(topology, depth_limit)
                    if result:
                        return result
                    
                    # C. UNDO (Backtrack)
                    # This is instant, unlike deepcopy
                    topology.remove_robot(new_id)

        return None
