from environment import Environment
from mcts import MCTSNode
from topology import SwarmTopology, visualize_ascii
import uuid

MAX_MODULES = 20

def form_topology(constraint: str):
    S = SwarmTopology(uuid.uuid4())
    initial_state = Environment(S, constraint, MAX_MODULES)
    root = MCTSNode(state=initial_state)

    final_topology = root.find_solution()

    return final_topology
