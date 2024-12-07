
TERRAIN_COSTS = {
    '.': 1,
    ';': 1.5,
    '+': 2.5,
    'x': 6,
    '@': -1 
}

import time
from collections import deque
import heapq
from utils import Utils

class Game:

    def __init__(self, width, height):

        self.row = width
        self.col = height
        self.map = [['' for _ in range(width)] for _ in range(height)]
        self.min_cost = float('inf')
        self.best_path = []


    def store_map(self, map_file):

        with open(map_file, encoding='UTF-8') as f:
            
            lines = f.readlines()[1:]  # Skip the first line with the dimensions
    
            # Populate the board with coordinates (X, Y)
            for x, line in enumerate(lines):  # `y` represents the row
                for y, char in enumerate(line.strip()):  # `x` represents the column
                    self.map[y][x] = char
        
         # print map
        for row in self.map:
            print(row)
        
    def find_best_path(self, alg, initial_x, initial_y, goal_x, goal_y):
        
        if alg == 'UCS':
            self.ucs(initial_x, initial_y, goal_x, goal_y)
        elif alg == 'BFS':
            self.bfs(initial_x, initial_y, goal_x, goal_y)
        elif alg == 'IDS':
            self.ids(initial_x, initial_y, goal_x, goal_y)
        elif alg == 'AStar':
            self.a_star(initial_x, initial_y, goal_x, goal_y)
        elif alg == 'Greedy':
            self.greedy(initial_x, initial_y, goal_x, goal_y)
        else:
            raise Exception(f'Algorithm {alg} not implemented')
        
        return self.best_path, self.min_cost

    """ def dfs(self, x, y, goal_x, goal_y, cost=0, path=None, visited=None):
        if visited is None:
            visited = set()
        if path is None:
            path = []

        # Out of bounds or already visited
        if x < 0 or y < 0 or x >= self.col or y >= self.row or (x, y) in visited or self.map[x][y] == '@':
            return
        
        print('Visiting', x, y , ':', self.map[x][y])

        # Add to the path and mark as visited
        path.append((x, y))
        visited.add((x, y))

        # Update the cost
        if len(path) != 1 : # no need to add the cost on the initial position
            cost += TERRAIN_COSTS[self.map[x][y]]

        # If we reached the goal, check if it's the best path
        if (x, y) == (goal_x, goal_y):
            if cost < self.min_cost:
                self.min_cost = cost
                self.best_path = path[:]
        else:
            # Explore all 4 directions
            directions = [(-1, 0), (1, 0), (0, -1), (0, 1)]
            for dx, dy in directions:
                self.dfs(x + dx, y + dy, goal_x, goal_y, cost, path, visited)

        # Backtrack: undo the move
        path.pop()
        visited.remove((x, y)) """

    
    def bfs(self, x, y, goal_x, goal_y, queue=None, visited=None):
        # Initialize queue and visited set on the first call
        if queue is None:
            queue = [(x, y, 0, [(x, y)])]
        if visited is None:
            visited = set()

        # Base case: stop recursion when the queue is empty
        if not queue:
            return

        # Dequeue the first node
        current_x, current_y, cost, path = queue.pop(0)

        # Skip if out of bounds, already visited, or blocked terrain
        if (
            current_x < 0 or current_y < 0 or 
            current_x >= self.col or current_y >= self.row or 
            (current_x, current_y) in visited or self.map[current_x][current_y] == '@'
        ):
            return self.bfs(x, y, goal_x, goal_y, queue, visited)

        #print('Visiting', current_x, current_y, ':', self.map[current_x][current_y])
        #print('Visited:', visited)
        #print('Path:', path)

        # Mark the node as visited
        visited.add((current_x, current_y))

        # Check if we reached the goal
        if (current_x, current_y) == (goal_x, goal_y):
            
            if cost < self.min_cost:
                self.min_cost = cost
                self.best_path = path[:]
        else:
            # Add neighbors to the queue with updated cost and path
            directions = [(-1, 0), (1, 0), (0, -1), (0, 1)]
            for dx, dy in directions:
                nx, ny = current_x + dx, current_y + dy
                new_cost = cost + (TERRAIN_COSTS[self.map[nx][ny]] if (0 <= nx < self.col and 0 <= ny < self.row and self.map[nx][ny] != '@') else 0)
                queue.append((nx, ny, new_cost, path + [(nx, ny)]))

        # Recursive call to process the next node in the queue
        self.bfs(x, y, goal_x, goal_y, queue, visited)

        path.remove((current_x, current_y))
        visited.remove((current_x, current_y))

    def dfs_limited(self, x, y, goal_x, goal_y, depth_limit, cost=0, path=None):
        if path is None:
            path = []

        # Out of bounds or blocked terrain
        if (
            x < 0 or y < 0 or x >= self.col or y >= self.row or 
            self.map[x][y] == '@' or len(path) > depth_limit
        ):
            return False

        # Add the current node to the path
        path.append((x, y))

        # Update the cost
        if len(path) != 1:  # No cost for the initial position
            cost += TERRAIN_COSTS[self.map[x][y]]

        # Check if we reached the goal
        if (x, y) == (goal_x, goal_y):
            if cost < self.min_cost:
                self.min_cost = cost
                self.best_path = path[:]
            path.pop()  # Backtrack
            return True

        # Explore all 4 directions
        directions = [(-1, 0), (1, 0), (0, -1), (0, 1)]
        for dx, dy in directions:
            if self.dfs_limited(x + dx, y + dy, goal_x, goal_y, depth_limit, cost, path):
                path.pop()  # Backtrack after exploring
                return True

        # Backtrack: undo the move
        path.pop()
        return False

    def ids(self, initial_x, initial_y, goal_x, goal_y):
        depth = 0
        while True:
            print(f"Trying depth limit: {depth}")
            if self.dfs_limited(initial_x, initial_y, goal_x, goal_y, depth):
                break
            depth += 1

        return self.best_path, self.min_cost

    def ucs(self, initial_x, initial_y, goal_x, goal_y):
        # Priority queue: stores (cumulative_cost, x, y, path)
        priority_queue = []
        heapq.heappush(priority_queue, (0, initial_x, initial_y, [(initial_x, initial_y)]))

        # Visited set: keeps track of visited nodes
        visited = set()

        while priority_queue:
            # Pop the node with the smallest cumulative cost
            current_cost, x, y, path = heapq.heappop(priority_queue)

            # If we've already visited this node, skip it
            if (x, y) in visited:
                continue

            # Mark the node as visited
            visited.add((x, y))

            print(f"Visiting: ({x}, {y}) with cost: {current_cost}")

            # Check if the goal has been reached
            if (x, y) == (goal_x, goal_y):
                print("Goal reached!")
                self.best_path = path
                self.min_cost = current_cost
                return path, current_cost

            # Explore neighbors
            directions = [(-1, 0), (1, 0), (0, -1), (0, 1)]
            for dx, dy in directions:
                nx, ny = x + dx, y + dy

                # Skip out-of-bounds or blocked terrain
                if 0 <= nx < self.col and 0 <= ny < self.row and self.map[nx][ny] != '@':
                    new_cost = current_cost + TERRAIN_COSTS[self.map[nx][ny]]
                    heapq.heappush(priority_queue, (new_cost, nx, ny, path + [(nx, ny)]))

        # If no path is found
        return None, float('inf')
    
    def a_star(self, initial_x, initial_y, goal_x, goal_y):

        utils = Utils()

        # Priority queue: stores (estimated_total_cost, cumulative_cost, x, y, path)
        priority_queue = []
        heapq.heappush(priority_queue, (0, 0, initial_x, initial_y, [(initial_x, initial_y)]))

        # Cost to reach each node
        g_costs = {(initial_x, initial_y): 0}

        while priority_queue:
            # Pop the node with the smallest estimated total cost
            _, current_cost, x, y, path = heapq.heappop(priority_queue)

            print(f"Visiting: ({x}, {y}) with cost: {current_cost}")

            # Check if the goal has been reached
            if (x, y) == (goal_x, goal_y):
                print("Goal reached!")
                self.best_path = path
                self.min_cost = current_cost

                return path, current_cost

            # Explore neighbors
            directions = [(-1, 0), (1, 0), (0, -1), (0, 1)]
            for dx, dy in directions:
                nx, ny = x + dx, y + dy

                # Skip out-of-bounds or blocked terrain
                if 0 <= nx < self.col and 0 <= ny < self.row and self.map[nx][ny] != '@':
                    # Calculate the new cost to reach this neighbor
                    new_cost = current_cost + TERRAIN_COSTS[self.map[nx][ny]]
                    
                    # Only consider this neighbor if it's a better path
                    if (nx, ny) not in g_costs or new_cost < g_costs[(nx, ny)]:
                        g_costs[(nx, ny)] = new_cost
                        heuristic = utils.heuristic_euclidean_distance(nx, ny, goal_x, goal_y)
                        estimated_total_cost = new_cost + heuristic
                        heapq.heappush(priority_queue, (estimated_total_cost, new_cost, nx, ny, path + [(nx, ny)]))

        # If no path is found
        return None, float('inf')

    def greedy(self, initial_x, initial_y, goal_x, goal_y):

        utils = Utils()
        # Priority queue: stores (heuristic_cost, x, y, path, real_cost)
        priority_queue = []
        heapq.heappush(priority_queue, (0, initial_x, initial_y, [(initial_x, initial_y)], 0))  # Include real_cost

        # Visited set to avoid revisiting nodes
        visited = set()

        while priority_queue:
            # Pop the node with the smallest heuristic cost
            _, current_x, current_y, path, real_cost = heapq.heappop(priority_queue)

            print(f"Visiting: ({current_x}, {current_y})")

            # Skip if out-of-bounds, blocked terrain, or already visited
            if (
                current_x < 0 or current_y < 0 or 
                current_x >= self.col or current_y >= self.row or 
                self.map[current_x][current_y] == '@' or 
                (current_x, current_y) in visited
            ):
                continue

            # Mark the node as visited
            visited.add((current_x, current_y))

            # Check if the goal is reached
            if (current_x, current_y) == (goal_x, goal_y):
                print("Goal reached!")
                self.best_path = path
                self.min_cost = real_cost
                return path, real_cost

            # Explore neighbors
            directions = [(-1, 0), (1, 0), (0, -1), (0, 1)]
            for dx, dy in directions:
                nx, ny = current_x + dx, current_y + dy

                # Skip out-of-bounds or blocked terrain
                if 0 <= nx < self.col and 0 <= ny < self.row and self.map[nx][ny] != '@':
                    # Calculate the real cost to the goal (include terrain cost)
                    new_real_cost = real_cost + TERRAIN_COSTS[self.map[nx][ny]]
                    # Calculate the heuristic cost to the goal (Euclidean distance)
                    heuristic = utils.heuristic_euclidean_distance(nx, ny, goal_x, goal_y)
                    heapq.heappush(priority_queue, (heuristic, nx, ny, path + [(nx, ny)], new_real_cost))

        # If no path is found
        return None, float('inf')