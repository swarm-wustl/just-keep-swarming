import math
import random
import copy

class MCTSNode:
    def __init__(self, state, parent=None, parent_action=None):
        self.state = state
        self.parent = parent
        self.parent_action = parent_action
        self.children = []
        self._number_of_visits = 0
        self._results = {}
        self._untried_actions = None

    @property
    def untried_actions(self):
        if self._untried_actions is None:
            self._untried_actions = self.state.get_legal_actions()
            random.shuffle(self._untried_actions)
        return self._untried_actions

    @property
    def q(self):
        wins = self._results.get(1, 0)
        losses = self._results.get(-1, 0)
        return wins - losses

    @property
    def n(self):
        return self._number_of_visits

    def expand(self):
        action = self.untried_actions.pop()
        next_state = self.state.move(action)
        child_node = MCTSNode(next_state, parent=self, parent_action=action)
        self.children.append(child_node)
        return child_node

    def is_terminal_node(self):
        return self.state.is_game_over()

    def rollout(self):
        sim_state = copy.deepcopy(self.state)
        
        while not sim_state.is_game_over():
            possible_moves = sim_state.get_legal_actions()
            
            if not possible_moves:
                # dead end
                return -1 
            
            action = random.choice(possible_moves)
            
            sim_state.apply_move(action)

            if sim_state.game_result() == 1:
                raise SolutionFound(copy.deepcopy(sim_state.topology))
            
        return sim_state.game_result()

    def backpropagate(self, result):
        self._number_of_visits += 1
        self._results[result] = self._results.get(result, 0) + 1
        if self.parent:
            self.parent.backpropagate(result)

    def is_fully_expanded(self):
        return len(self.untried_actions) == 0

    def best_child(self, c_param=1.414):
        log_n = math.log(self.n)
        
        choices_weights = [
            (child.q / child.n) + c_param * math.sqrt((2 * log_n / child.n))
            for child in self.children
        ]
        return self.children[choices_weights.index(max(choices_weights))]

    def _tree_policy(self):
        current_node = self
        while not current_node.is_terminal_node():
            if not current_node.is_fully_expanded():
                return current_node.expand()
            else:
                current_node = current_node.best_child()
        return current_node

    def best_action(self):
        simulation_no = 1000
        
        for _ in range(simulation_no):
            v = self._tree_policy()
            reward = v.rollout()
            v.backpropagate(reward)
        
        return self.best_child(c_param=0)

    def find_solution(self, simulations=1000):
        """
        Replaces 'best_action'. 
        Runs simulations and returns the winning topology immediately if found.
        """
        try:
            for _ in range(simulations):
                v = self._tree_policy()
                reward = v.rollout()
                v.backpropagate(reward)
        except SolutionFound as e:
            print("  -> Simulation found a valid structure! Stopping search.")
            return e.winning_topology
        
        print("  -> Search exhausted without finding full solution.")
        return None

class SolutionFound(Exception):
    def __init__(self, topology):
        self.winning_topology = topology
