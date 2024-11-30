
TERRAIN_COSTS = {
    '.': 1,
    ';': 1.5,
    '+': 2.5,
    'x': 6,
    '@': -1 
}

class Game:

    def __init__(self, width, height):

        self.width = width
        self.height = height
        self.map = [['' for x in range(width)] for y in range(height)]
    
    
