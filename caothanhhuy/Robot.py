import math

from scipy.spatial import KDTree

from Spot import*


def dist_real(point1, point2):
    (x1, y1) = point1
    (x2, y2) = point2
    x1 = float(x1)
    x2 = float(x2)
    y1 = float(y1)
    y2 = float(y2)

    px = (x1 - x2) ** 2
    py = (y1 - y2) ** 2
    distance = (px + py) ** 0.5
    return distance


class Robot:
    def __init__(self, startPos, robotImg, width):

        self.x, self.y = startPos
        self.theta = 0
        self.width = width
        self.vr = 20  # right velocity
        self.vl = 20  # left velocity
        self.u = (self.vl + self.vr)/2  # linear velocity
        self.max_u = 30
        self.min_u = 10
        self.W = 0  # angular velocity
        self.a = 15  # width of robot
        self.trail_set = []
        self.dt = 0  # time step
        self.pathRb = []
        self.angle = []
        one_degree = math.pi / 180
        self.angle.append(0)
        for i in range(1, 60, 4):
            self.angle.append(i * one_degree)
            self.angle.append(-i * one_degree)
        self.img = pygame.image.load(robotImg)
        self.img = pygame.transform.scale(self.img, (16, 16))
        self.rotated = self.img
        self.rect = self.rotated.get_rect(center=(self.x, self.y))

    def move(self, event=None):
        self.x += ((self.vl + self.vr)/2)*math.cos(self.theta)*self.dt
        self.y += ((self.vl + self.vr)/2)*math.sin(self.theta)*self.dt
        self.theta += (self.vr - self.vl)/self.width*self.dt
        self.rotated = pygame.transform.rotozoom(
            self.img, math.degrees(-self.theta), 1)
        self.rect = self.rotated.get_rect(center=(self.x, self.y))

        self.following()

    def following(self):
        target = self.pathRb[0]
        delta_x = target[0] - self.x
        delta_y = target[1] - self.y

        self.u = delta_x * math.cos(self.theta) + \
            delta_y * math.sin(self.theta)
        self.W = (-1/self.a) * math.sin(self.theta) * delta_x + \
            (1/self.a) * math.cos(self.theta)*delta_y

        self.vr = self.u + (self.W * self.width)/2
        self.vl = self.u - (self.W * self.width)/2
        if dist_real((self.x, self.y), target) < 20 and len(self.pathRb) > 1:
            self.pathRb.pop(0)

    def chage_linear_velocity(self, delta_u):
        self.u = self.u + delta_u
        if self.u > self.max_u:
            self.u = self.max_u
        elif self.u < self.min_u:
            self.u = self.min_u

    def move_to(self, angle, win, grid, target, total_dt, end):
        global time_replan
        time_move = pygame.time.get_ticks()
        # FPS = self.dt /50
        self.W = angle

        self.vr = self.u + (self.W * self.width)/2
        self.vl = self.u - (self.W * self.width)/2
        # print("vr: ", self.vr, "vl: ", self.vl, "W: ", self.W, "u: ", self.u)
        last_time = pygame.time.get_ticks()
        while pygame.time.get_ticks() - time_move < 1500 and dist_real((self.x, self.y), target) > 10:

            if (total_dt > 0.5):
                total_dt = 0
            time_step = (pygame.time.get_ticks() - last_time) / 1000
            self.dt = time_step
            last_time = pygame.time.get_ticks()
            self.x += ((self.vl + self.vr)/2)*math.cos(self.theta)*self.dt
            self.y += ((self.vl + self.vr)/2)*math.sin(self.theta)*self.dt
            self.theta += (self.vr - self.vl)/self.width*self.dt/1.5

            # self.theta += self.W / 400

            if self.theta >= math.pi * 2 or self.theta <= -2*math.pi:
                self.theta = 0

            self.rotated = pygame.transform.rotozoom(
                self.img, math.degrees(-self.theta), 1)
            self.rect = self.rotated.get_rect(center=(self.x, self.y))
            self.draw(win)
            self.trail((self.x, self.y), win, GREEN)
            total_dt += (pygame.time.get_ticks() - last_time) / 1000
            pygame.display.update()

            # check obstacle

        return total_dt

    def find_spot_with_angle(self, grid, angle, win):
        gap = 10
        dt = 1.5
        theta = self.theta + angle
        x = self.x + ((self.vl + self.vr)/2)*math.cos(theta)*dt
        y = self.y + ((self.vl + self.vr)/2)*math.sin(theta)*dt
        if x > 800 or x < 0 or y > 800 or y < 0:
            print("out of range")
            return None, None, None
        row, col = int(x / gap), int(y / gap)
        row2, col2 = int((x + 10) / gap), int(y / gap)
        row3, col3 = int(x / gap), int((y + 10) / gap)
        row4, col4 = int((x + 12) / gap), int((y + 12) / gap)
        row5, col5 = int((x - 12) / gap), int((y - 12) / gap)
        row6, col6 = int((x - 10) / gap), int(y / gap)
        row7, col7 = int(x / gap), int((y - 10) / gap)
        row8, col8 = int((x + 12) / gap), int((y - 12) / gap)
        row9, col9 = int((x - 12) / gap), int((y + 12) / gap)

        spot = grid[row][col]
        # pygame.draw.line(win, RED, (self.x, self.y), (x, y), 1)
        # pygame.display.update()
        if spot.is_barrier() or grid[row2][col2].is_barrier() or grid[row3][col3].is_barrier() or grid[row4][col4].is_barrier() or grid[row5][col5].is_barrier() or grid[row6][col6].is_barrier() or grid[row7][col7].is_barrier() or grid[row8][col8].is_barrier() or grid[row9][col9].is_barrier():
            return None, None, None

        time_global = pygame.time.get_ticks()
        return spot, x, y

    def find_next_spot(self, grid, target, win, distance, kdTree: KDTree):
        current = self.x, self.y
        gap = 800/80
        # the next point will be based on velocity, time and angle
        if current != target:
            neighbors = []
            for i in range(len(self.angle)):
                spot, x, y = self.find_spot_with_angle(
                    grid, self.angle[i], win)
                if spot is not None:
                    neighbors.append((spot, x, y, self.angle[i]))
            # print(len(neighbors))
            if len(neighbors) > 0:
                # find the spot with the smallest distance to the target
                min_spot = None
                obstacle_dist = 0
                f_cost = 100000
                for i in range(len(neighbors)):
                    # dist = distance[neighbors[i][0]]
                    dist = dist_real(
                        (neighbors[i][1], neighbors[i][2]), target)
                    nearest_obstacle_dist, _ = kdTree.query(
                        [(neighbors[i][1], neighbors[i][2])])
                    nearest_core = 1 / (nearest_obstacle_dist + 1)
                    if nearest_core * 250 + dist < f_cost:
                        f_cost = nearest_core * 250 + dist
                        min_spot = neighbors[i][0]
                        angle_selected = neighbors[i][3]
                        obstacle_dist = nearest_obstacle_dist

                return min_spot, angle_selected, obstacle_dist
            else:
                return None, None, None

    def move_back(self, grid, win, total_dt):
        time_move = pygame.time.get_ticks()
        # FPS = self.dt /50
        vr_prev = self.vr
        vl_prev = self.vl
        self.vr = -self.u / 2
        self.vl = -self.u / 2
        last_time = pygame.time.get_ticks()
        while pygame.time.get_ticks() - time_move < 2000:
            if (total_dt > 0.5):
                total_dt = 0
            time_step = (pygame.time.get_ticks() - last_time) / 1000
            self.dt = time_step
            last_time = pygame.time.get_ticks()
            self.x += ((self.vl + self.vr)/2)*math.cos(self.theta)*self.dt
            self.y += ((self.vl + self.vr)/2)*math.sin(self.theta)*self.dt
            # self.theta -= (self.vr - self.vl)/self.width*self.dt/1.5

            if self.theta >= math.pi * 2 or self.theta <= -2*math.pi:
                self.theta = 0

            self.rotated = pygame.transform.rotozoom(
                self.img, math.degrees(-self.theta), 1)
            self.rect = self.rotated.get_rect(center=(self.x, self.y))
            self.draw(win)
            self.trail((self.x, self.y), win, GREEN)
            total_dt += (pygame.time.get_ticks() - last_time) / 1000
            pygame.display.update()
        self.vr = vr_prev
        self.vl = vl_prev
        return total_dt

    def draw(self, map):
        map.blit(self.rotated, self.rect)

    def trail(self, pos, map, color):
        for i in range(0, len(self.trail_set) - 1):
            pygame.draw.line(map, color, (self.trail_set[i][0], self.trail_set[i][1]),
                             (self.trail_set[i+1][0], self.trail_set[i+1][1]), 3)
        self.trail_set.append(pos)