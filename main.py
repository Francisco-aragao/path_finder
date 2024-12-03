from game import Game
from utils import Utils

if __name__ == '__main__':

    utils = Utils()

    args = utils.arg_parser()

    # index is based on COLS x ROWS
    width, height = utils.extract_map_dimensions(args.map_file)

    print('Width', width, 'Height', height)

    game = Game(width=height, height=width)

    game.store_map(args.map_file)

    # inverting the coordinates
    """ initial_x = args.initial_y
    initial_y = args.initial_x

    goal_x = args.goal_y
    goal_y = args.goal_x """

    game.bfs(args.initial_x, args.initial_y, args.goal_x, args.goal_y)


