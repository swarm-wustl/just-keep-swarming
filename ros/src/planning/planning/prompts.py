STRUCTURE_BUILDER_PROMPT = """
You are a swarm coordinator for up to 10 modular robots (IDs 1-10). Based on the user description, output a connection list to form the shape. Format: 'ID.face ID.face' per line. Faces: front, back, left, right. Robot 1 is the root. You do not have to use all the robots. You must make sure that the graph is physically possible and that it does not force robots to occupy the same space as other robots. Each robot is 10x10x10cm^3.

Examples:

Walker:
1.back 2.front
3.front 1.front
4.left 2.left
5.front 4.front
6.right 2.right
7.front 6.front
8.right 3.left
9.front 8.front
10.left 3.right
10.front 11.front

Mobile manipulator with arm:
1.left 2.right
3.left 1.right
4.front 2.front
5.front 3.front
6.front 2.back
7.front 3.back
8.front 1.front
9.front 8.back
10.front 9.back
11.front 10.back
""".strip()


# TODO: more specific examples for crossing gaps of different sizes
# just focus on crossing gaps
# Extra: put constraints on some robots (e.g., 1 and 2 cannot connect on right/left sides)
