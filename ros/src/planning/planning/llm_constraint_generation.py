from openai import OpenAI
import os
from search import form_topology, visualize_ascii
from dotenv import load_dotenv

load_dotenv()

openai_api_key = os.getenv("OPENAI_API_KEY")
if not openai_api_key:
    raise ValueError("OPENAI_API_KEY env var not set")

client = OpenAI(api_key=openai_api_key)

SYSTEM_PROMPT = """
You are the planner for a modular robot swarm. Each module is a **10x10cm** square.
Task: Analyze the environment and output a single Python boolean expression defining the robot's shape `S`.

**CRITICAL HEURISTICS:**
1.  **Unit Conversion:** You MUST convert map distances (cm) to Module Units (10cm).
    * *Gap of 40cm* -> Requires `S.length >= 5` (4 units + 1 overlap).
    * *Passage of 30cm* -> Requires `S.width <= 2` (Must fit inside 3 units).
    * *Reach target 20cm away* -> Requires `S.radius >= 2` or `S.diameter >= 4`.

2.  **Pruning (Speed):** MCTS solves faster if you constrain unused dimensions.
    * If you just need a line, specify `S.width == 1`.
    * If you need a symmetric block, specify `S.width <= 4` rather than leaving it open.

3.  **Topology Logic:**
    * **Crossing:** Use `S.length` and `S.is_aligned` (Linear stability).
    * **Straddling:** Use `S.width` and `S.is_symmetric_y` (Lateral balance).
    * **Multi-Goal:** Use `S.endpoints` and `S.diameter` (Branching reach).

**Available Parameters:**
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

**Output Format:**
Return ONLY the constraint string.

**Examples:**
- *Bridge a 20cm gap:* `S.length >= 3 and S.width <= 2 and S.is_aligned`
- *Straddle parallel rails 30cm apart:* `S.width >= 4 and S.is_symmetric_y`
- *Touch 4 points 20cm from center:* `S.endpoints >= 4 and S.diameter >= 5`
"""

def generate_constraint(environment_description: str) -> str:
    response = client.chat.completions.create(
        model="gpt-4o",
        messages=[
            {"role": "system", "content": SYSTEM_PROMPT},
            {"role": "user", "content": environment_description}
        ],
        temperature=0.2
    )

    content = response.choices[0].message.content.strip()
    constraint = extract_code_block(content)

    return constraint 

def generate_topology(environment_description: str):
    constraint = generate_constraint(environment_description)
    print(f"LLM generated constraint: {constraint}")

    print(f"Forming a topology to meet constraint...")
    topology = form_topology(constraint)

    return topology

import re

def extract_code_block(llm_output: str) -> str:
    pattern = r"```(?:[\w]*\n)?(.*?)```"
    
    match = re.search(pattern, llm_output, re.DOTALL)
    
    if match:
        return match.group(1).strip()
    return None
