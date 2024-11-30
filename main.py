from game import Game
from utils import Utils

if __name__ == '__main__':

    utils = Utils()

    args = utils.arg_parser()

    # index is based on COLS x ROWS
    width, height = utils.extract_map_dimensions(args.map_file)

    game = Game(width, height)



