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
    