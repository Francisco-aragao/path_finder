from game import Game
from utils import Utils
from time import time
import sys
sys.setrecursionlimit(2000000) # raise the recursion limit to avoid recursion errors

if __name__ == '__main__':

    utils = Utils()

    args = utils.arg_parser()

    # index is based on COLS x ROWS so the width is the height and vice versa
    width, height = utils.extract_map_dimensions(args.map_file)
    game = Game(width=height, height=width)
    game.map = utils.store_map(map_file=args.map_file, width=height, height=width)

    start = time()
    path, cost, expanded_nodes = game.find_best_path(args.algorithm, args.initial_x, args.initial_y, args.goal_x, args.goal_y)
    end = time()

    # printing default output
    print(cost, end=' ')
    for p in path:
        print(p, end=' ')
    print()

    # printing additional information to measure performance
    if args.measure:
        print('Time:', end - start)
        print('Expanded nodes:', expanded_nodes)
