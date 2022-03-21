from queue import PriorityQueue
import cv2
import numpy as np
import map_file
import random

def ol_dict_add(node):
    open_list_dict[str(node["node_position"][0]) + "," + \
                   str(node["node_position"][1]) + "," + \
                   str(node["node_theta"])] = node

def ol_pq_add(node):
    open_list_pq.put((node["c2c"] + node["c2g"] + random.random() * 0.001, node))

def cost_to_goal(node_position, goal_position):
    return np.sqrt(pow(goal_position[0] - node_position[0], 2) + pow(goal_position[1] - node_position[1], 2))

def check_move(position):
    print(position[0], position[1])
    if position[0] < 0   or \
       position[0] > 399 or \
       position[1] < 0   or \
       position[1] > 249:
        return False
    # else:
    # if clearance_map[int(np.ceil(position[1])), int(np.ceil(position[0]))] == 0:
    #     return False
    # elif clearance_map[int(np.floor(position[1])), int(np.floor(position[0]))] == 0:
    #     return False
    if   clearance_map[249-position[1], position[0]] == 0:
        return False
    elif clearance_map[249-position[1], position[0]] == 0:
        return False
    else:
        return True

def action(theta, current_node):
    new_theta = (current_node["node_theta"] + theta) % 360
    new_position = int(current_node["node_position"][0]+step_size*np.cos(np.deg2rad(new_theta))), \
                    int(current_node["node_position"][1]+step_size*np.sin(np.deg2rad(new_theta)))
    
    if not check_move(new_position):
        return
    
    new_c2c = current_node["c2c"] + 1
    new_c2g = cost_to_goal(new_position, goal_position)

    # uniqueness check
    match_node = open_list_dict.get(str(new_position[0]) + "," + \
                                    str(new_position[1]) + "," + \
                                    str(new_theta), False)

    if match_node:
        # update cost
        if match_node["c2c"] + match_node["c2g"] > new_c2c + new_c2g:
            pass
        # do nothing
        else:
            pass
    else:
        # add a new node to open_list
        new_node = {
        "node_index": 0,
        "parent_node_index": "TBD",
        "node_position": new_position,
        "node_theta": new_theta,
        "c2c": new_c2c, 
        "c2g": new_c2g
        }
        print(new_position, new_c2c, new_c2g)
        clearance_map[249-new_position[1], new_position[0]] = 0
        # cv2.imshow("img", clearance_map)
        # cv2.waitKey(0)
        ol_dict_add(new_node)
        ol_pq_add(new_node)


def action_ll(current_node):
   action(60, current_node)
def action_l(current_node):
   action(30, current_node)
def action_s(current_node):
   action(0, current_node)
def action_r(current_node):
   action(-30, current_node)
def action_rr(current_node):
   action(-60, current_node)

# ================================= #
#    #     #    #    ### #     #    #
#    ##   ##   # #    #  ##    #    #
#    # # # #  #   #   #  # #   #    #
#    #  #  # #     #  #  #  #  #    #
#    #     # #######  #  #   # #    #
#    #     # #     #  #  #    ##    #
#    #     # #     # ### #     #    #
# ================================= #
                             

# start and goal position
# euclidian x, y
start_position = 10, 200
sx, sy = start_position
start_theta = 30
# start_theta = np.deg2rad(start_theta)

# euclidian x, y
goal_position = 250, 70
gx, gy = goal_position
goal_theta = 30
# goal_theta = np.deg2rad(goal_theta)

# game parameters
obstacle_clearance = 5
robot_radius = 5
step_size = 7

# start node and goal node
start_node = {
    "node_index":0,
    "parent_node_index": "root",
    "node_position": start_position,
    "node_theta": start_theta,
    "c2c": 0, 
    "c2g": np.sqrt(pow(goal_position[0] - start_position[0], 2) + pow(goal_position[1] - start_position[1], 2))
}



# # clearance_map = map_file.make_map(clearance=1)
clearance_map = map_file.create_map(clearance=5)
cv2.circle(clearance_map, (sx, 249-sy), 2, (0, 0, 0), -1)
cv2.circle(clearance_map, (gx, 249-gy), 2, (0, 0, 0), -1)

open_list_dict = {}
open_list_pq = PriorityQueue()
config_space = np.array((500, 800, 12), np.uint8)
closed_list = list()

ol_dict_add(start_node)
ol_pq_add(start_node)

# LOOP
for i in range(100):
    print("------------------")
    print(i)
    print("------------------")
    # pick lowest total cost node
    _, current_node = open_list_pq.get()
    # current_node = start_node

    # move to this node

    # update visited conf space

    action_ll(current_node)
    action_l(current_node)
    action_s(current_node)
    action_r(current_node)
    action_rr(current_node)
    print(len(open_list_dict))

    # move ll
    # move l
    # move s
    # move r
    # move rr

    # repeat










cv2.imshow("img", clearance_map)
cv2.waitKey(0)
cv2.destroyAllWindows()