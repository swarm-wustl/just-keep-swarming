from typing import List, Tuple
import numpy as np
import heapq
import math
from collections import defaultdict
from math import inf

State = Tuple[int, int]

def manhattan(a: State, b: State) -> int:
    return abs(a[0] - b[0]) + abs(a[1] - b[1])

class PriorityQueue:
    def __init__(self):
        self.heap = []
        self.entry_finder = {}
        self.REMOVED = '<removed>'
        self.counter = 0

    def insert(self, state: State, key):
        if state in self.entry_finder:
            self.remove(state)
        entry = [key, self.counter, state]
        self.counter += 1
        self.entry_finder[state] = entry
        heapq.heappush(self.heap, entry)

    def remove(self, state: State):
        entry = self.entry_finder.pop(state, None)
        if entry is not None:
            entry[-1] = self.REMOVED

    def top_key(self):
        while self.heap:
            key, _, state = self.heap[0]
            if state != self.REMOVED:
                return key
            heapq.heappop(self.heap)
        return (inf, inf)

    def pop(self):
        while self.heap:
            _, _, state = heapq.heappop(self.heap)
            if state != self.REMOVED:
                self.entry_finder.pop(state, None)
                return state
        raise KeyError("Pop from empty")

    def __contains__(self, state):
        return state in self.entry_finder

    def __len__(self):
        return len(self.entry_finder)

class DStarLitePlanner:
    def __init__(self, grid_ref: np.ndarray, s_start: State, s_goal: State):
        # STORE REFERENCE to grid instead of copying so it can update extrenally
        self.grid = grid_ref 
        self.s_start = s_start
        self.s_goal = s_goal

        self.H, self.W = self.grid.shape 
        
        self.U = PriorityQueue()
        self.km = 0.0
        self.g = defaultdict(lambda: inf)
        self.rhs = defaultdict(lambda: inf)

        self.rhs[self.s_goal] = 0.0
        self.U.insert(self.s_goal, self._calculate_key(self.s_goal))
        self.replan()
        self.s_last = self.s_start

    def _in_bounds(self, s: State) -> bool:
        x, y = s
        h, w = self.grid.shape
        return 0 <= x < w and 0 <= y < h

    def _is_obstacle(self, s: State) -> bool:
        x, y = s
        return self.grid[y, x] == 1

    def _succ(self, u: State):
        x, y = u
        nbrs = []
        # 8-connected movement
        for dx in (-1, 0, 1):
            for dy in (-1, 0, 1):
                if dx == 0 and dy == 0: continue
                v = (x + dx, y + dy)
                if self._in_bounds(v) and not self._is_obstacle(v):
                    nbrs.append(v)
        return nbrs

    def _cost(self, u: State, v: State):
        if self._is_obstacle(u) or self._is_obstacle(v):
            return inf
        # Euclidean cost for 8-connected grid
        return math.hypot(u[0] - v[0], u[1] - v[1])

    def _calculate_key(self, s: State):
        h = manhattan(self.s_start, s)
        return (min(self.g[s], self.rhs[s]) + h + self.km, min(self.g[s], self.rhs[s]))

    def _update_vertex(self, u: State):
        if u != self.s_goal:
            best = inf
            for sp in self._succ(u):
                val = self._cost(u, sp) + self.g[sp]
                if val < best: best = val
            self.rhs[u] = best
        if u in self.U: self.U.remove(u)
        if self.g[u] != self.rhs[u]: self.U.insert(u, self._calculate_key(u))

    def replan(self):
        while (len(self.U) > 0 and self.U.top_key() < self._calculate_key(self.s_start)) \
                or (self.rhs[self.s_start] != self.g[self.s_start]):
            if len(self.U) == 0: break
            k_old = self.U.top_key()
            u = self.U.pop()
            if k_old < self._calculate_key(u):
                self.U.insert(u, self._calculate_key(u))
            elif self.g[u] > self.rhs[u]:
                self.g[u] = self.rhs[u]
                for s in self._succ(u): self._update_vertex(s)
            else:
                self.g[u] = inf
                for s in self._succ(u) + [u]: self._update_vertex(s)

    def update_start(self, new_start: State):
        if new_start != self.s_start:
            self.km += manhattan(self.s_last, new_start)
            self.s_last = self.s_start
            self.s_start = new_start
            # TODO: detect map changes and update edge costs
            # (current this is relying on replan being called)

    def extract_path(self) -> List[State]:
        """Greedily extracts the path from start to goal based on current G values."""
        path = [self.s_start]
        curr = self.s_start
        
        # Simple timeout
        max_steps = self.grid.size 
        steps = 0

        while curr != self.s_goal and steps < max_steps:
            steps += 1
            best = inf
            nxt = None
            for s in self._succ(curr):
                val = self._cost(curr, s) + self.g[s]
                if val < best:
                    best = val
                    nxt = s
            
            if nxt is None or best == inf:
                break # No path found
            
            curr = nxt
            path.append(curr)

        return path
