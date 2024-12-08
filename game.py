from collections import deque
import heapq
from utils import Utils


TERRAIN_COSTS = {
    '.': 1,
    ';': 1.5,
    '+': 2.5,
    'x': 6,
    '@': -1 
}

# variables to store the number of expanded nodes and the costs -> not important for the solution, just for analysis
EXPANDED_NODES = 0
costs = []

class Game:
    """
    Class to represent the game map and find the path using different algorithms
    """

    def __init__(self, width, height):

        self.row = width
        self.col = height
        self.map = [['' for _ in range(width)] for _ in range(height)]
        self.min_cost = float('inf')
        self.best_path = []

    # main function to call each algorithm
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
        
        return self.best_path, self.min_cost, EXPANDED_NODES

    
    def bfs(self, initial_x, initial_y, goal_x, goal_y, queue=None, visited=set()):

    
        if queue is None: # queue = (initial coordinates, cost, path)
            queue = [(initial_x, initial_y, 0, [(initial_x, initial_y)])]

        if not queue:
            return

        # get next info
        current_x, current_y, cost, path = queue.pop(0)

        # skip if out of bounds, already visited, or blocked terrain (@)
        if (
            current_x < 0 or current_y < 0 or 
            current_x >= self.col or current_y >= self.row or 
            (current_x, current_y) in visited or self.map[current_x][current_y] == '@'
        ):
            return self.bfs(initial_x, initial_y, goal_x, goal_y, queue, visited)
        
        global EXPANDED_NODES
        EXPANDED_NODES += 1

        visited.add((current_x, current_y))

        # check if we reached the goal
        if (current_x, current_y) == (goal_x, goal_y):
            
            if cost < self.min_cost:
                self.min_cost = cost
                self.best_path = path[:]
        else:
            
            # check each direction for neighbors
            directions = [(-1, 0), (1, 0), (0, -1), (0, 1)]
            for dx, dy in directions:
                nx, ny = current_x + dx, current_y + dy
                new_cost = cost + (TERRAIN_COSTS[self.map[nx][ny]] if (0 <= nx < self.col and 0 <= ny < self.row and self.map[nx][ny] != '@') else 0) # dont add invalid terrains
                queue.append((nx, ny, new_cost, path + [(nx, ny)]))

        # recursive call to process the next node in the queue
        self.bfs(initial_x, initial_y, goal_x, goal_y, queue, visited)

        # alows revisiting previous nodes, backtracking
        path.remove((current_x, current_y))
        visited.remove((current_x, current_y))

    
    def dfs_limited(self, initial_x, initial_y, goal_x, goal_y, max_depth, visited=set(), depth=0, path=[]):

        # stop search when reaching the max depth
        if depth > max_depth:
            return []
        
        if (initial_x, initial_y) in visited:
            return []

        visited.add((initial_x, initial_y))

        # test goal
        if (initial_x, initial_y) == (goal_x, goal_y):
            new_path = path + [(initial_x, initial_y)]
            return new_path 
        
        global EXPANDED_NODES
        EXPANDED_NODES += 1
        

        directions = [(-1, 0), (1, 0), (0, -1), (0, 1)]
        for dx, dy in directions:
            nx, ny = initial_x + dx, initial_y + dy
            
            # skip invalid positions before start a new seach
            if (
                nx < 0 or ny < 0 or 
                nx >= self.col or ny >= self.row or 
                (nx, ny) in visited or self.map[nx][ny] == '@'
            ):
                continue
            
            # explore recursively each neighbor
            new_path = self.dfs_limited(nx, ny, goal_x, goal_y, max_depth, visited, depth + 1, path + [(initial_x, initial_y)])
            
            # if a path is found, return it
            if new_path:
                return new_path
        
        # if no path found, backtrack
        visited.remove((initial_x, initial_y))
        return None



    def ids(self, initial_x, initial_y, goal_x, goal_y):
        depth = 0
        while True:
            
            path = self.dfs_limited(initial_x, initial_y, goal_x, goal_y, depth)

            # stop IDS if a path is found or the depth is greater than the map size
            if path or depth > self.row * self.col:
                self.best_path = path
                self.min_cost = sum(TERRAIN_COSTS[self.map[x][y]] for x, y in path)
                break

            depth += 1
            

        return self.best_path, self.min_cost

    def ucs(self, initial_x, initial_y, goal_x, goal_y):

        # priority queue: (cumulative_cost, x, y, path)
        priority_queue = []
        heapq.heappush(priority_queue, (0, initial_x, initial_y, [(initial_x, initial_y)]))

        visited = set()

        while priority_queue:
            
            # get the node with the smallest cumulative cost
            current_cost, current_x, current_y, path = heapq.heappop(priority_queue)

            if (current_x, current_y) in visited:
                continue

            global EXPANDED_NODES
            EXPANDED_NODES += 1

            visited.add((current_x, current_y))


            # check goal
            if (current_x, current_y) == (goal_x, goal_y):
                self.best_path = path
                self.min_cost = current_cost
                return path, current_cost

            # check neighbors
            directions = [(-1, 0), (1, 0), (0, -1), (0, 1)]
            for dx, dy in directions:
                nx, ny = current_x + dx, current_y + dy

                # skip invalid positions
                if 0 <= nx < self.col and 0 <= ny < self.row and self.map[nx][ny] != '@':
                    new_cost = current_cost + TERRAIN_COSTS[self.map[nx][ny]]
                    heapq.heappush(priority_queue, (new_cost, nx, ny, path + [(nx, ny)]))

        # if no path is found
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
            _, current_cost, current_x, current_y, path = heapq.heappop(priority_queue)
            
            # skip if out-of-bounds, blocked terrain, or already visited
            if (
                current_x < 0 or current_y < 0 or 
                current_x >= self.col or current_y >= self.row or 
                self.map[current_x][current_y] == '@'
            ):
                continue

            global EXPANDED_NODES
            EXPANDED_NODES += 1

            # check goal
            if (current_x, current_y) == (goal_x, goal_y):
                self.best_path = path
                self.min_cost = current_cost

                return path, current_cost

            # explore neighbors
            directions = [(-1, 0), (1, 0), (0, -1), (0, 1)]
            for dx, dy in directions:
                nx, ny = current_x + dx, current_y + dy

                # Skip invalid positions
                if 0 <= nx < self.col and 0 <= ny < self.row and self.map[nx][ny] != '@':
                    
                    # Calculate the new (REAL) cost to reach this neighbor
                    new_cost = current_cost + TERRAIN_COSTS[self.map[nx][ny]]
                    
                    # update the cost if it is the first time visiting this node or the new REAL cost is lower
                    if (nx, ny) not in g_costs or new_cost < g_costs[(nx, ny)]:
                        g_costs[(nx, ny)] = new_cost
                        heuristic = utils.heuristic_euclidean_distance(nx, ny, goal_x, goal_y)
                        estimated_total_cost = new_cost + heuristic
                        heapq.heappush(priority_queue, (estimated_total_cost, new_cost, nx, ny, path + [(nx, ny)]))

        # if no path is found
        return None, float('inf')

    def greedy(self, initial_x, initial_y, goal_x, goal_y):

        utils = Utils()
        # Priority queue: stores (heuristic_cost, x, y, path, real_cost)
        priority_queue = []
        heapq.heappush(priority_queue, (0, initial_x, initial_y, [(initial_x, initial_y)], 0)) 

        visited = set()

        while priority_queue:

            # get the node with the smallest heuristic cost
            _, current_x, current_y, path, real_cost = heapq.heappop(priority_queue)

            # skip if out-of-bounds, blocked terrain, or already visited
            if (
                current_x < 0 or current_y < 0 or 
                current_x >= self.col or current_y >= self.row or 
                self.map[current_x][current_y] == '@' or 
                (current_x, current_y) in visited
            ):
                continue

            costs.append(real_cost)
            
            global EXPANDED_NODES
            EXPANDED_NODES += 1

            visited.add((current_x, current_y))

            # Check if the goal is reached
            if (current_x, current_y) == (goal_x, goal_y):
                self.best_path = path
                self.min_cost = real_cost
                return path, real_cost

            # explore neighbors
            directions = [(-1, 0), (1, 0), (0, -1), (0, 1)]
            for dx, dy in directions:
                nx, ny = current_x + dx, current_y + dy

                # skip invalid positions
                if 0 <= nx < self.col and 0 <= ny < self.row and self.map[nx][ny] != '@':
                    
                    # the real cost is stored to be returned in the end, it is not used on the algorithm decisions
                    new_real_cost = real_cost + TERRAIN_COSTS[self.map[nx][ny]]

                    # calculate the heuristic cost to reach this neighbor
                    heuristic = utils.heuristic_euclidean_distance(nx, ny, goal_x, goal_y)
                    heapq.heappush(priority_queue, (heuristic, nx, ny, path + [(nx, ny)], new_real_cost))

        # if no path is found
        return None, float('inf')