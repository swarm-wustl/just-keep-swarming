from llm_constraint_generation import generate_topology
from topology import visualize_ascii

environment1 = """
The robot is operating in a 500cm x 500cm square area defined as Polygon([(0, 0), (500, 0), (500, 500), (0, 500)]). You cannot travel outside of the area.

There are chasm obstacles at: `{Polygon([(0, 240), (500, 240), (500, 260), (0, 260)])}`.

There are solid obstacles at: `{Polygon([(0, 260), (240, 260), (240, 270), (0, 270)]), Polygon([(275, 260), (500, 260), (500, 270), (275, 270)])}`

Your current position is `Point(250, 0)` and your goal is to reach `Point(250, 500)`.
"""

environment2 = """
The robot is operating in a 500cm x 500cm square area defined as Polygon([(0, 0), (500, 0), (500, 500), (0, 500)]). You cannot travel outside of the area.

There are chasm obstacles at: {}.

There are solid obstacles at: {Polygon([(0, 0), (200, 0), (200, 200), (0, 200)]), Polygon([(300, 0), (500, 0), (500, 200), (300, 200)]), Polygon([(0, 300), (200, 300), (200, 500), (0, 500)]), Polygon([(300, 300), (500, 300), (500, 500), (300, 500)])}.

Your current position is Point(250, 250) (Center).

Your goals (must be touched simultaneously) are: [Point(250, 230), Point(250, 270), Point(230, 250), Point(270, 250)].
"""

topology1 = generate_topology(environment1)
if topology1 is not None:
    print(topology1.robots) 
    print(topology1.get_observation()) 
    visualize_ascii(topology1)
else:
    print("Failed to generate topology 1")

topology2 = generate_topology(environment2)
if topology2 is not None:
    print(topology2.robots) 
    print(topology2.get_observation()) 
    visualize_ascii(topology2)
else:
    print("Failed to generate topology 2")
