from search import form_topology
from topology import SwarmTopology, visualize_ascii
import time

# constraint = "S.size < 5 and S.length > 3"
# constraint = "S.max_degree <= 2 and S.length >= 5 and S.width == 1"
# constraint = "S.size >= 3 and S.width <= 2"
# constraint = "S.length >= 3 and S.width <= 2 and S.is_aligned == True"
# constraint = "S.width >= 3 and S.length >= 3 and S.is_aligned and S.max_degree == 4"
# constraint = "S.width >= 4 and S.length >= 3 and S.is_symmetric_x and S.max_degree <= 2"
# constraint = "S.width >= 4 and S.length >= 3 and S.is_symmetric_x"
# constraint = "S.length >= 5 and S.width <= 3 and S.is_aligned"
# constraint = "S.width <= 1 and S.is_aligned"
# constraint = "S.length >= 6 and S.width >= 2"
# constraint = "S.endpoints == 4 and S.size == 10 and S.is_symmetric_x and S.endpoints_aligned"
# constraint = "S.length >= 3 and S.width <= 5"
# constraint = "S.width <= 5"
# constraint = "S.width == 1"
constraint = "S.length >= 3 and S.width == 1 and S.is_aligned"
# constraint = "S.is_aligned"

print(constraint)
start_mcts = time.time()
final_topology = form_topology(constraint)
end_mcts = time.time()
print(f"MCTS time: {end_mcts - start_mcts}")

if final_topology is not None:
    print("\nSUCCESS: Structure completed!")
    print(f"Final Size: {len(final_topology.robots)}")
    # This is the object you want:
    print(final_topology.robots) 
    print(final_topology.get_observation()) 
    visualize_ascii(final_topology)
else:
    print("\nFAILURE: Could not satisfy constraint within module limits.")
