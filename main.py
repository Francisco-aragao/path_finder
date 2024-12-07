from game import Game
from utils import Utils
from time import time

if __name__ == '__main__':

    utils = Utils()

    args = utils.arg_parser()

    # index is based on COLS x ROWS
    width, height = utils.extract_map_dimensions(args.map_file)

    print('Width', width, 'Height', height)

    game = Game(width=height, height=width)

    game.map = utils.store_map(map_file=args.map_file, width=height, height=width)

    # inverting the coordinates
    """ initial_x = args.initial_y
    initial_y = args.initial_x

    goal_x = args.goal_y
    goal_y = args.goal_x """

    # store initial time   
    start = time()
    path, cost, expanded_nodes = game.find_best_path(args.algorithm, args.initial_x, args.initial_y, args.goal_x, args.goal_y)
    end = time()

    print(cost, end=' ')
    for p in path:
        print(p, end=' ')

    print()

    if args.measure:
        print('Time:', end - start)
        print('Expanded nodes:', expanded_nodes)
