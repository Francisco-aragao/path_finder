import argparse
import os

class Utils:

    def arg_parser(self):
        parser = argparse.ArgumentParser(description="""Path Finder. To run, follow the example: 
            python main.py <map file> <algorithm> <initial x> <initial y> <goal x> <goal y>
                                        
            The algorithm can be one of the following: BFS, IDS, UCS, Greedy, AStar
        where:
        """)
        parser.add_argument('map_file', type=str, help='File containing the map')
        parser.add_argument('algorithm', type=str, help='Algorithm to use', choices=['BFS', 'IDS', 'UCS', 'Greedy', 'AStar'])
        parser.add_argument('initial_x', type=int, help='Initial x coordinate')
        parser.add_argument('initial_y', type=int, help='Initial y coordinate')
        parser.add_argument('goal_x', type=int, help='Goal x coordinate')
        parser.add_argument('goal_y', type=int, help='Goal y coordinate')
        
        parser.add_argument('--measure', action='store_true', help='Measure time')

        args = parser.parse_args()

        if not os.path.isfile(args.map_file):
            parser.error(f"File {args.map_file} not found")
        
        return args

    def extract_map_dimensions(self, map_file):

        with open(map_file, encoding='UTF-8') as f:
            dimensions = f.readline().split()
            width = int(dimensions[0])
            height = int(dimensions[1])

        return width, height
    
    def heuristic_euclidean_distance(self, x1, y1, x2, y2):
        return ((x1 - x2) ** 2 + (y1 - y2) ** 2) ** 0.5
    
    def heuristic_manhattan_distance(self, x1, y1, x2, y2):
        return abs(x1 - x2) + abs(y1 - y2)
    
    def store_map(self, map_file, width, height):

        game_map = [['' for _ in range(width)] for _ in range(height)]

        with open(map_file, encoding='UTF-8') as f:
            
            lines = f.readlines()[1:]  # Skip the first line with the dimensions
    
            # Populate the board with coordinates (X, Y)
            for x, line in enumerate(lines):  # `y` represents the row
                for y, char in enumerate(line.strip()):  # `x` represents the column
                    game_map[y][x] = char
        
         # print map
        """ for row in game_map:
            print(row) """
        
        return game_map