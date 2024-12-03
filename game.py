
TERRAIN_COSTS = {
    '.': 1,
    ';': 1.5,
    '+': 2.5,
    'x': 6,
    '@': -1 
}

import time

class Game:

    def __init__(self, width, height):

        self.row = width
        self.col = height
        self.map = [['' for _ in range(width)] for _ in range(height)]


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
        

    def dfs(self, initial_x, initial_y, goal_x, goal_y):
        
        # It is not optimal

        stack = [(initial_x, initial_y, 0, [])]
        visited = set()
        min_cost = float('inf')
        min_path = []

        while stack:

            x, y, cost, path = stack.pop()

            if (x, y) in visited or x >= self.col or y >= self.row or x < 0 or y < 0:
                continue

            print('Visiting', x, y , ':', self.map[x][y])

            visited.add((x, y))

            path.append((x, y))

            if (x, y) == (goal_x, goal_y):
                if cost < min_cost:
                    min_cost = cost
                    min_path = path
                continue

            cost += TERRAIN_COSTS[self.map[x][y]]

            directions = [(-1, 0), (1, 0), (0, -1), (0, 1)]

            for dx, dy in directions:
                stack.append((x + dx, y + dy, cost, path.copy()))

        print(min_path)
        print(min_cost)
        return min_path, min_cost
    

    def bfs(self, initial_x, initial_y, goal_x, goal_y):
        
        # Implement the Breadth First Search algorithm to find the shortest path

        queue = [(initial_x, initial_y, 0, [])]
        visited = set()
        min_cost = float('inf')
        min_path = []

        while queue:

            x, y, cost, path = queue.pop(0)

            
            if (x, y) in visited or x >= self.col or y >= self.row or x < 0 or y < 0:
                continue

            print('Visiting', x, y , ':', self.map[x][y])

            visited.add((x, y))

            path.append((x, y))

            if (x, y) == (goal_x, goal_y):
                if cost < min_cost:
                    min_cost = cost
                    min_path = path
                continue

            cost += TERRAIN_COSTS[self.map[x][y]]

            directions = [(-1, 0), (1, 0), (0, -1), (0, 1)]

            for dx, dy in directions:
                queue.append((x + dx, y + dy, cost, path.copy()))
            
            visited.remove((x, y))

        print(min_path)
        print(min_cost)
        return min_path, min_cost