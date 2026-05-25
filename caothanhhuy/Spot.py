import pygame

RED = (255, 0, 0)
GREEN = (0, 255, 0)
YELLOW = (255, 255, 0)
WHITE = (255, 255, 255)
BLACK = (0, 0, 0)
PURPLE = (128, 0, 128)
ORANGE = (255, 165, 0)
GREY = (128, 128, 128)
TURQUOISE = (0, 0, 205)

ORANGE1 = (255, 165, 0)
ORANGE2 = (250, 164, 96)
BLUE1 = (0, 191, 255)
BLUE2 = (135, 206, 235)
GREEN1 = (0, 255, 127)
GREEN2 = (144, 238, 144)
PINK1 = (255, 182, 193)
PINK2 = (255, 228, 225)
BROWN1 = (205, 133, 63)
BROWN2 = (222, 184, 135)

CLOSED1 = (65, 105, 225)
CLOSED2 = (0, 191, 255)
CLOSED3 = (135, 206, 235)
CLOSED4 = (176, 224, 230)


class Spot:
    def __init__(self, row, col, width, total_rows, safe):
        self.row = row
        self.col = col
        self.x = row * width
        self.y = col * width
        self.color = WHITE
        self.neighbors = []
        self.width = width
        self.total_rows = total_rows
        self.safe = safe

    def get_pos(self):
        return self.row, self.col

    def get_row(self):
        return self.row

    def get_col(self):
        return self.col

    def get_x(self):
        return self.x + 4

    def get_y(self):
        return self.y + 4

    def get_safe(self):
        return self.safe

    def is_barrier(self):
        return self.color == BLACK

    def is_open(self):
        return self.color == GREEN

    def is_closed(self):
        return self.color == ORANGE

    def is_start(self):
        return self.color == RED

    def is_end(self):
        return self.color == TURQUOISE

    def is_open_safe(self):
        return self.color == GREEN

    def is_closed_safe(self):
        return self.color == ORANGE

    def reset(self):
        self.color = WHITE

    def make_start(self):
        self.color = RED

    # def make_closed(self, n):
    #     if n % 5 == 0:
    #         self.color = ORANGE1
    #     elif n % 5 == 1:
    #         self.color = GREEN1
    #     elif n % 5 == 2:
    #         self.color = BROWN1
    #     elif n % 5 == 3:
    #         self.color = PINK1
    #     elif n % 5 == 4:
    #         self.color = BLUE1

    def make_closed(self, n):
        if n % 5 == 0:
            self.color = WHITE
        elif n % 5 == 1:
            self.color = WHITE
        elif n % 5 == 2:
            self.color = WHITE
        elif n % 5 == 3:
            self.color = WHITE
        elif n % 5 == 4:
            self.color = WHITE

    # def make_open(self, n):
    #     if n % 5 == 0:
    #         self.color = ORANGE2
    #     elif n % 5 == 1:
    #         self.color = GREEN2
    #     elif n % 5 == 2:
    #         self.color = BROWN2
    #     elif n % 5 == 3:
    #         self.color = PINK2
    #     elif n % 5 == 4:
    #         self.color = BLUE2

    def make_open(self, n):
        if n % 5 == 0:
            self.color = WHITE
        elif n % 5 == 1:
            self.color = WHITE
        elif n % 5 == 2:
            self.color = WHITE
        elif n % 5 == 3:
            self.color = WHITE
        elif n % 5 == 4:
            self.color = WHITE

    def make_open_safe(self):
        self.color = GREEN

    def make_closed_safe(self, n):
        if n < 1.71:
            self.color = WHITE
        elif n < 4:
            self.color = WHITE
        elif n < 8:
            self.color = WHITE
        else:
            self.color = WHITE

    def make_barrier(self):
        self.color = BLACK

    def make_end(self):
        self.color = TURQUOISE

    def make_path(self):
        self.color = PURPLE

    def draw(self, win):
        pygame.draw.rect(win, self.color, (self.x, self.y, self.width, self.width))

    def update_neighbors(self, grid):
        self.neighbors = []
        if self.row < self.total_rows - 1 and not grid[self.row + 1][self.col].is_barrier():  # DOWN
            self.neighbors.append(grid[self.row + 1][self.col])

        if self.row > 0 and not grid[self.row - 1][self.col].is_barrier():  # UP
            self.neighbors.append(grid[self.row - 1][self.col])

        if self.col < self.total_rows - 1 and not grid[self.row][self.col + 1].is_barrier():  # RIGHT
            self.neighbors.append(grid[self.row][self.col + 1])

        if self.col > 0 and not grid[self.row][self.col - 1].is_barrier():  # LEFT
            self.neighbors.append(grid[self.row][self.col - 1])

        if self.row > 0 and self.col > 0 and not grid[self.row - 1][self.col - 1].is_barrier():
            self.neighbors.append(grid[self.row - 1][self.col - 1])  # Up and left

        if self.row > 0 and self.col < self.total_rows - 1 and not grid[self.row - 1][self.col + 1].is_barrier():
            self.neighbors.append(grid[self.row - 1][self.col + 1])  # up and right

        if self.row < self.total_rows - 1 and self.col > 0 and not grid[self.row + 1][self.col - 1].is_barrier():
            self.neighbors.append(grid[self.row + 1][self.col - 1])  # down and left

        if self.row < self.total_rows - 1 and self.col < self.total_rows - 1 and not grid[self.row + 1][self.col + 1].is_barrier():
            self.neighbors.append(grid[self.row + 1][self.col + 1])  # down and right

        return self.neighbors

    def __lt__(self, other):
        return False