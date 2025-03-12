from q1_env import Env
import numpy as np
from collections import deque

# Read input from the user
rows = []
while True:
    line = input().strip()
    if line == "":  # Stop on an empty line
        break
    rows.append([int(x) for x in line.split()])

# Convert list of lists into a NumPy array
matrix = np.array(rows)

env = Env(matrix.copy())  # creating environment
# pathgrid = env.matrix.copy() * -1 # used for visualization
t = 0  # to count no. of moves
n = 2  # to generate id of move (helps in finding moves which led to minimum time path)
dir_s = 0 # starting direction of bot  (-1 -> right, 1 -> left, 0 -> front)

# a double ended queue of nodes.
# nodes are points from where we choose action
q = deque([(env.end_pos.copy(), env.speed, env.direction.copy(), 0, 0, 0, t, 0)])  # [pos, speed, direction, n, d, theta, t, dir_s]
m = set()  # memory to store visited states
m.add((env.pos[0], env.pos[1], env.speed, env.direction[0], env.direction[1]))  # (y, x, speed, dir_y, dir_x)
g = {}  # graph of positions (to backtrack moves which led to minimum time path)
# g= {id1 : [(id2, a),....], id2 : [(id10, a')......}
# a is action which led to new id from initial id

q.append((env.end_pos.copy(), env.speed, np.array([0,-1]), 1, 0, 0, t, 1))
q.append((env.end_pos.copy(), env.speed, np.array([0,1]), 2, 0, 0, t, -1)) # all possible starting directions
while q:  # while q is non-empty

    # d is to check whether bot has moved
    # theta is the total angle moved from initial pos
    # theta is positive if bot has moved clockwise else negative
    pos, speed, direction, nt, d, theta, t, dirn = q.popleft()  # saving values and removing the node from q

    # theta should be around 2*pi (6.28) for complete clockwise loop
    if np.array_equal(pos, env.end_pos) and t != 0 and d and theta > 6:  #if current pos = end pos and bot has traveled
        # print(d, t, nt, dirn)
        # print(direction, speed, theta)
        if speed ==1:
            t += 1 # last move is speedD so t increments
            print('Minimum Moves to complete Clockwise Loop:', t)
            dir_s = dirn
            n = nt  #saving id of move
            break
        elif speed == 0:
            print('Minimum Moves to complete Clockwise Loop:', t)
            dir_s = dirn
            n = nt  # saving id of move
            break

    t += 1

    g[nt] = []
    for k1 in (9, 5, 6): # choose speed change
        for k2 in (9, 1, 2, 3, 4): # choose to turn
            if speed > 0 and k2 != 9:
                continue
            for k3 in (7, 8, 9):  # choose lateral movement
                dt =0
                # if k2 != 9: # in case lateral movement is not allowed while turning
                #     k3 = 9
                # Create copies to avoid modifying the environment state directly
                pos_copy, speed_copy, direction_copy = pos.copy(), speed, direction.copy()
                env.pos, env.speed, env.direction = pos_copy, speed_copy, direction_copy
                env.theta = theta
                k = (k1, k2, k3) # tuple of actions to take in this move
                if env.step(k):  # takes given action and if it is valid (return value is True)
                    state = (env.pos[0], env.pos[1], env.speed, env.direction[0], env.direction[1])
                    # checks if current state is in memory, i.e, already visited state
                    if state not in m:
                        n += 1
                        if (k2 != 9 and k2 != 5) or k3 != 9 :  #if i > 3 then the bot has definitely moved
                            dt = 1
                        # pathgrid[env.pos[0], env.pos[1]] = t
                        q.append((env.pos.copy(), env.speed, env.direction.copy(), n, d+ dt, env.theta, t, dirn))  # adds new node to q
                        g[nt].append((n, k, env.pos.copy().tolist()))  # adds id and action to graph
                        m.add(state)  # adds state to memory


# function to extract all actions taken in minimum time path
def backtrack_key(graph, start_key):
    action_array = deque()
    pos_array = deque()
    current_key = start_key
    while True:
        found = False
        for key, values in graph.items():
            for val1, val2, val3 in values:
                if val1 == current_key:
                    action_array.appendleft(val2)  # Save the action
                    pos_array.appendleft(val3) # Save the position
                    current_key = key  # Move to the new key
                    found = True
                    break
            if found:
                break
        if not found:
            break  # Stop if no more backtracking is possible

    return action_array, pos_array


# Start backtracking from final id (n)
result_action, result_pos = backtrack_key(g, n)
print()
print('Action Sequence:', result_action)
# print('Position Sequence:', result_pos)
env2 = Env(matrix.copy())  # new environment
if dir_s == 1:
    env2.direction = np.array([0,-1])
elif dir_s == -1:
    env2.direction = np.array([0,1])
else:
    env2.direction = np.array([-1,0])
fa_matrix = env2.matrix.copy()  # matrix having path of bot
for action in result_action:
    env2.step(action)
    fa_matrix[env2.pos[0], env2.pos[1]] = 3

# fp_matrix is just for cross-checking
# fp_matrix = env2.matrix.copy()
# for y,x in result_pos:
#     fp_matrix[y, x] = 3
print()
print('Path:')
print(fa_matrix)
# print(fp_matrix)
