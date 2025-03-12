import numpy as np

#making environment
class Env:
    def __init__(self, matrix):
        #given map
        self.matrix = matrix

        self.speed = 0
        #current direction is kept upwards
        self.direction = np.array([-1,0])  #[y,x]

        # a dictionary which maps numbers to action to take
        # easy to use when iterating
        self.action_map = {
            1 : self.dirLeft0,
            2 : self.dirRight0,
            3 : self.dirLeft1,
            4 : self.dirRight1,
            5 : self.speedD,
            6 : self.speedI,
            7 : self.moveLeft,
            8 : self.moveRight,
            9 : self.idle
        }

        self.pos = np.array(np.where(self.matrix == 3)).T[0]  # [y,x] # starting position
        self.end_pos = self.pos.copy() # end position (same as start position)
        self.matrix[self.pos[0], self.pos[1]] = 0 # making starting position makes sure that it is not treated as obstacle
        self.r = np.array([[0, -1], [1, 0]]) # right rotation matrix
        self.l = np.array([[0, 1], [-1, 0]]) # left rotation matrix
        self.theta =0 # clockwise angle moved

    def dirRight0(self):
        self.direction = np.dot(self.direction, self.r) #rightwards rotation of direction
    def dirLeft0(self):
        self.direction = np.dot(self.direction, self.l) #leftwards rotation of direction
    def dirRight1(self):
        self.direction = np.dot(self.direction, self.r)
        self.speedI()
    def dirLeft1(self):
        self.direction = np.dot(self.direction, self.l) #leftwards rotation of direction
        self.speedI()
    def speedI(self):
        self.speed += 1
    def speedD(self):
        self.speed -= 1
    def moveRight(self):
        self.pos += np.dot(self.direction, self.r)
    def moveLeft(self):
        self.pos += np.dot(self.direction, self.l)
    def idle(self):
        self.speed += 0

    def step(self, keys): #takes a given action in environment
        old_pos = self.pos.copy()
        m, n = self.matrix.shape
        for key in keys:
            self.action_map[key]()
        self.pos += self.direction * self.speed

        # to make sure bot is moving clockwise we take vector cross product of final pos vector and old pos vector
        # vectors are taken wrt center of matrix
        # if old pos vector (v2) X final pos vector (v1) is positive implies clockwise movement
        # negative implies anticlockwise movement
        # angle moved is given by cp/|v1||v2|

        v1 = [self.pos[0] - m/2, self.pos[1] - n/2] # [y1, x1]
        v2 = [old_pos[0] - m/2, old_pos[1] - n/2] # [y2, x2]
        cp = v1[0]*v2[1] - v2[0]*v1[1] # v2 X v1 = x2*y1 - y2*x1
        mod_v1 = ((self.pos[0] - m/2)**2 + (self.pos[1] - n/2)**2)**0.5
        mod_v2 = ((old_pos[0] - m/2)**2 + (old_pos[1] - n/2)**2)**0.5
        theta = cp/(mod_v1*mod_v2)
        self.theta += theta

        if self.theta < 0: # discarding move if bot is moving anticlockwise
            return False
        if keys == (9,9,9) and self.speed == 0: # discard idle move
            return False
        # discarding moves which make bot out of boundary
        if self.pos[0] < 0 or self.pos[0] > m - 1 or self.pos[1] < 0 or self.pos[1] > n - 1:
            return False

        # discard move if bot moves out of lane or crashes with obstacle
        min_y, max_y = sorted([old_pos[0], self.pos[0]])
        min_x, max_x = sorted([old_pos[1], self.pos[1]])
        submatrix = self.matrix[min_y:max_y + 1, min_x:max_x + 1] # a matrix consisting of points between old pos and final pos
        if np.any(submatrix): # if any point in submatrix is True, i.e, 1 or 2
            return False

        # discard move if speed turns negative
        if self.speed < 0:
            return False

        return True
