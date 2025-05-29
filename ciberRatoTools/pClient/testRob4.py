import sys
import numpy as np
import heapq
from croblink import *
from math import *
from enum import Enum
import math
import xml.etree.ElementTree as ET
import os

CELL_ROWS=7
CELL_COLS=14

center = 0
left = 1
right = 2
back = 3

MAPPING_COLS = (((CELL_COLS * 2) - 1) * 2) + 1
MAPPING_ROWS = (((CELL_ROWS * 2) - 1) * 2) + 1

######################################################################################
class SearchDomain:

    def actions(self, state):
        raise NotImplementedError

    def result(self, state, action):
        raise NotImplementedError

    def cost(self, state, action):
        return 1

    def heuristic(self, state, goal):
        return 0

class SearchNode:
    def __init__(self, state, parent, action=None, depth=0, cost=0, heuristic=0): 
        self.state = state
        self.parent = parent
        self.action = action 
        self.depth = depth
        self.cost = cost
        self.heuristic = heuristic
        self.f = self.cost + self.heuristic 
        
    def in_parent(self, state):
        if self.parent is None:
            return False
        return self.parent.state == state or self.parent.in_parent(state)

class SearchProblem:
    def __init__(self, domain, initial, goal):
        self.domain = domain
        self.initial = initial
        self.goal = goal
    
    def goal_test(self, state):
        return state == self.goal

class SearchTree:
    
    def __init__(self, problem, strategy='a_star'):
        self.problem = problem
        self.root = SearchNode(
            state=problem.initial, 
            parent=None, 
            action=None, 
            depth=0, 
            cost=0,
            heuristic=self.problem.domain.heuristic(problem.initial, problem.goal)
        )
        self.count = 0
        self.open_nodes = []
        heapq.heappush(self.open_nodes, (self.root.f, self.count, self.root))
        self.strategy = strategy
        self.cost = 0
        self.length = 0

    def get_path(self, node):
        if node.parent == self.root or node.parent == None:
            return [node.state]
        path = self.get_path(node.parent)
        path += [node.state]
        return path

    def search(self, limit):
        while self.open_nodes:
            _, _, current_node = heapq.heappop(self.open_nodes)
            self.cost += current_node.cost
            self.length += 1
            if self.problem.goal_test(current_node.state):
                return self.get_path(current_node), current_node.cost, current_node.depth
            elif self.length >= limit:
                return self.get_path(current_node), current_node.cost, current_node.depth
            for action in self.problem.domain.actions(current_node.state):
                child_state = self.problem.domain.result(current_node.state, action)
                if not current_node.in_parent(child_state) and current_node.depth < limit:
                    new_current_node = SearchNode(
                        child_state,
                        current_node,
                        action,
                        current_node.depth + 1,
                        current_node.cost + self.problem.domain.cost(current_node.state, action),
                        self.problem.domain.heuristic(child_state, self.problem.goal)
                    )
                    self.count += 1
                    heapq.heappush(self.open_nodes, (new_current_node.f, self.count, new_current_node))
        return None

    def add_to_open(self, lnewnodes):
        if self.strategy == 'a_star':
            self.open_nodes = sorted(self.open_nodes + lnewnodes,
                                   key=lambda node: node.heuristic + node.cost)

######################################################################################

class Domain(SearchDomain):

    def __init__(self, connections):
        self.connections = connections

    def actions (self, cell):
        actions = []
        for (c1, c2) in self.connections:
            if c1 == cell:
                actions.append((c1, c2))   
            elif c2 == cell: 
                actions.append((c2, c1))
        return actions

    def result(self, cell, action):
        (c1, c2) = action
        if c1 == cell:
             return c2
             

    def cost (self, state, action):
        return 1

    def heuristic(self, state, goal):
        return math.hypot(state[0]-goal[0], state[1]-goal[1])

class MyRob(CRobLinkAngs):
    def __init__(self, rob_name, rob_id, angles, host, mapfile, pathfile):
        CRobLinkAngs.__init__(self, rob_name, rob_id, angles, host)
        self.lap_time = 0
        self.readSensors()

        self.errorPrev = 0

        self.curr_x = 0
        self.curr_y = 0
        self.prev_x = 0
        self.prev_y = 0
        self.prev_lpout = 0
        self.prev_rpout = 0
        self.curr_theta = 0
        self.prev_theta = 0
        self.curr_gps = [0, 0]

        self.map = [[' ' for col in range(0, MAPPING_COLS)] for row in range(0, MAPPING_ROWS)]
        self.curr_mapping = None
        self.init_mapping = None

        self.curr_cell = None
        self.next_cell = None

        self.newtowalk_cells = []
        self.newtowalk_neighborhood = []
        self.neighborhood = []
        self.visited_cells = []

        self.map_connections = []

        self.tree_path = []
        self.beacon_positions = {}
        self.beacon_mapping = {}  
        self.beacons = 0
        self.beacon = 1
        self.beacon_path = []
        self.inicial_x = 0
        self.inicial_y = 0
        self.beaconCanWrite = True
        self.beaconCanWriteSimTime = True   

        self.mapfile = mapfile
        self.pathfile = pathfile 

        self.planning_done = False

        MAP = []
        for i in range(27):
            rows = 55 * [10]
            MAP.append(rows)

        MAP_x_current = MAP_y_current = []


    # In this map the center of cell (i,j), (i in 0..6, j in 0..13) is mapped to labMap[i*2][j*2].
    # to know if there is a wall on top of cell(i,j) (i in 0..5), check if the value of labMap[i*2+1][j*2] is space or not
    def setMap(self, labMap):
        self.labMap = labMap

    def printMap(self):
        for l in reversed(self.labMap):
            print(''.join([str(l) for l in l]))

    def run(self):
        if self.status != 0:
            print("Connection refused or error")
            quit()

        state = 'stop'
        stopped_state = 'walk'
        angle = 0
        cardinal = 0

        print("Starting robot...")
        self.readSensors()
        print("Initial sensors read")
        self.inicial_x = self.measures.x
        self.inicial_y = self.measures.y
        print(f"Initial position: ({self.inicial_x}, {self.inicial_y})")
        self.Define_position()
        print("Initial position defined")

        if os.path.exists(self.pathfile):
            os.remove(self.pathfile)

        while True:
            self.readSensors()
            print(f"Current state: {state}")

            if self.measures.endLed:
                print(f"{self.rob_name} exiting")
                quit()

            if state == 'stop' and self.measures.start:
                print("Transitioning to running state")
                state = stopped_state

            if state != 'stop' and self.measures.stop:
                print("Transitioning to stopped state")
                stopped_state = state
                state = 'stop'

            if state == 'choose':
                print("Choosing next move...")
                state, angle, cardinal = self.Move(state)
                print(f"Chose: state={state}, angle={angle}, cardinal={cardinal}")

            if state == "walk":
                state = self.move_one()

            if state == "rot_left":
                state = self.Rotate_90_left(angle, cardinal)

            if state == "rot_right":
                state = self.Rotate_90_right(angle, cardinal)

            if state=='end':
                self.driveMotors(0.0,0.0)
                for beacon_id in self.beacon_mapping:
                    x, y = self.beacon_mapping.get(beacon_id)
                    self.map[x][y] = beacon_id    
                if os.path.exists(self.mapfile):
                    os.remove(self.mapfile)
                with open(self.mapfile, 'w') as mapfile:
                    for row in self.map[::-1]:
                        mapfile.write(''.join([str(a) for a in row]) + '\n')
                self.write_planning()
                sys.exit()

            if self.measures.time > int(self.simTime)-50 and self.beaconCanWriteSimTime:     
                for i in self.beacon_mapping:
                    x, y = self.beacon_mapping.get(i)
                    self.map[x][y] =  str(i)   
                if os.path.exists(self.mapfile):
                    os.remove(self.mapfile)
                with open(self.mapfile, 'w') as mapfile:
                    for row in self.map[::-1]:
                        mapfile.write(''.join([str(a) for a in row]) + '\n') 
                self.write_planning()
                self.beaconCanWriteSimTime = False  

    def path_finding(self, start_cell, goal_cell):
        domain = Domain(self.map_connections)
        problem = SearchProblem(domain, start_cell, goal_cell)
        search_tree = SearchTree(problem, strategy='a_star')
        result = search_tree.search(limit=2500)
        if result:
            path = result[0]
            return path
        else:
            return []

    def Define_position(self):
        self.curr_mapping = [int(MAPPING_ROWS/2), int(MAPPING_COLS/2)] 
        self.init_mapping = self.curr_mapping

        self.map[self.curr_mapping[0]][self.curr_mapping[1]] = '0'

        self.curr_cell = [0, 0]
        if self.curr_cell not in self.visited_cells:
            self.visited_cells.append(self.curr_cell)

        neighbors =  self.check_neighborhood()
        self.newtowalk_cells.extend(neighbors)
        self.neighborhood = self.newtowalk_cells
        self.newtowalk_neighborhood = self.newtowalk_cells

        self.map_connections = [[self.curr_cell, neighbor_cell] for neighbor_cell in self.neighborhood]

    def DefineQuadrant(self):
        compass_v = self.measures.compass
        cardinal_dir = None
        center = left = right = back = None
    
        if -45 <= compass_v < 45:
            cardinal_dir = 'N'
            center = [0, 1]
            left = [1, 0]
            back = [0, -1]
            right = [-1, 0]
        elif 45 <= compass_v < 135:
            cardinal_dir = 'O'
            center = [1, 0]
            left = [0, -1]
            back = [-1, 0]
            right = [0, 1]
        elif compass_v >= 135 or compass_v < -135:
            cardinal_dir = 'S'
            center = [0, -1]
            left = [-1, 0]
            back = [0, 1]
            right = [1, 0]
        elif -135 <= compass_v < -45:
            cardinal_dir = 'E'
            center = [-1, 0]
            left = [0, 1]
            back = [1, 0]
            right = [0, -1]
    
        return cardinal_dir, center, left, right, back

    def check_neighborhood(self):
        center_id, left_id, right_id, back_id = 0, 1, 2, 3
        cardinal_dir, center, left, right, back = self.DefineQuadrant()
        neighborhood = []
    
        directions = [
            (center_id, center),
            (back_id, back),
            (left_id, left),
            (right_id, right)
        ]
    
        for sensor_id, direction in directions:
            dx, dy = direction
            map_x = self.curr_mapping[0] + dx
            map_y = self.curr_mapping[1] + dy
            cell_x = self.curr_cell[0] + dy * 2
            cell_y = self.curr_cell[1] + dx * 2
    
            sensor_value = self.measures.irSensor[sensor_id]
            if sensor_value > 1.4:
                if cardinal_dir in ['N', 'S']:
                    symbol = '|' if direction in [center, back] else '-'
                else:
                    symbol = '-' if direction in [center, back] else '|'
                self.map[map_x][map_y] = symbol

            elif sensor_value < 1:
                self.map[map_x][map_y] = 'X'
                neighborhood.append([cell_x, cell_y])
    
        return neighborhood

    def update_self_gps(self, diff, cardinal):

        if (cardinal == "N" or cardinal == "S"):
            self.curr_mapping = [self.curr_mapping[0], self.curr_mapping[1]+diff]
        else:
            self.curr_mapping = [self.curr_mapping[0]+diff, self.curr_mapping[1]]

        if (cardinal == "N" or cardinal == "S"):
            self.curr_cell = [self.curr_cell[0]+diff, self.curr_cell[1]]
        else:
            self.curr_cell = [self.curr_cell[0], self.curr_cell[1]+diff]

        self.map[self.curr_mapping[0]][self.curr_mapping[1]] = 'X'

        self.visited_cells.append(self.curr_cell)  if self.curr_cell not in self.visited_cells else print("Already Visited")

        neighbors =  self.check_neighborhood()

        self.newtowalk_cells.extend([c for c in neighbors if c not in self.newtowalk_cells and c not in self.visited_cells])

        for c in self.newtowalk_cells:
            if c in self.visited_cells:             
                self.newtowalk_cells.remove(c)

        self.newtowalk_cells.sort(key=lambda new_cell: abs(math.dist(new_cell, self.curr_cell))) 

        self.neighborhood = neighbors

        self.newtowalk_neighborhood = [fcell for fcell in self.neighborhood if fcell in self.newtowalk_cells] 

        if cardinal == "N":
            if (closest_cell:=[self.curr_cell[0], self.curr_cell[1]-2]) in self.newtowalk_neighborhood:
                self.newtowalk_cells.remove(closest_cell)
                self.newtowalk_cells = [closest_cell] + self.newtowalk_cells
            elif (closest_cell:=[self.curr_cell[0]+2, self.curr_cell[1]]) in self.newtowalk_neighborhood:
                self.newtowalk_cells.remove(closest_cell)
                self.newtowalk_cells = [closest_cell] + self.newtowalk_cells
            elif (closest_cell:=[self.curr_cell[0], self.curr_cell[1]+2]) in self.newtowalk_neighborhood:
                self.newtowalk_cells.remove(closest_cell)
                self.newtowalk_cells = [closest_cell] + self.newtowalk_cells
        elif cardinal == "S":
            if (closest_cell:=[self.curr_cell[0], self.curr_cell[1]+2]) in self.newtowalk_neighborhood:
                self.newtowalk_cells.remove(closest_cell)
                self.newtowalk_cells = [closest_cell] + self.newtowalk_cells
            elif (closest_cell:=[self.curr_cell[0]-2, self.curr_cell[1]]) in self.newtowalk_neighborhood:
                self.newtowalk_cells.remove(closest_cell)
                self.newtowalk_cells = [closest_cell] + self.newtowalk_cells
            elif (closest_cell:=[self.curr_cell[0], self.curr_cell[1]-2]) in self.newtowalk_neighborhood:
                self.newtowalk_cells.remove(closest_cell)
                self.newtowalk_cells = [closest_cell] + self.newtowalk_cells
        elif cardinal == "E":
            if (closest_cell:=[self.curr_cell[0]-2, self.curr_cell[1]]) in self.newtowalk_neighborhood:
                self.newtowalk_cells.remove(closest_cell)
                self.newtowalk_cells = [closest_cell] + self.newtowalk_cells
            elif (closest_cell:=[self.curr_cell[0], self.curr_cell[1]-2]) in self.newtowalk_neighborhood:
                self.newtowalk_cells.remove(closest_cell)
                self.newtowalk_cells = [closest_cell] + self.newtowalk_cells
            elif (closest_cell:=[self.curr_cell[0]+2, self.curr_cell[1]]) in self.newtowalk_neighborhood:
                self.newtowalk_cells.remove(closest_cell)
                self.newtowalk_cells = [closest_cell] + self.newtowalk_cells

        else:
            if (closest_cell:=[self.curr_cell[0]+2, self.curr_cell[1]]) in self.newtowalk_neighborhood:
                self.newtowalk_cells.remove(closest_cell)
                self.newtowalk_cells = [closest_cell] + self.newtowalk_cells
            elif (closest_cell:=[self.curr_cell[0], self.curr_cell[1]+2]) in self.newtowalk_neighborhood:
                self.newtowalk_cells.remove(closest_cell)
                self.newtowalk_cells = [closest_cell] + self.newtowalk_cells
            elif (closest_cell:=[self.curr_cell[0]-2, self.curr_cell[1]]) in self.newtowalk_neighborhood:
                self.newtowalk_cells.remove(closest_cell)
                self.newtowalk_cells = [closest_cell] + self.newtowalk_cells


        self.map_connections = self.map_connections + [[self.curr_cell, neighbor_cell] for neighbor_cell in self.neighborhood
                                                                                            if [self.curr_cell, neighbor_cell] not in self.map_connections
                                                                                            and [neighbor_cell, self.curr_cell] not in self.map_connections]    
    
    def Move(self, state):
        cardinal, _, _, _, _ = self.DefineQuadrant()
    
        if self.measures.ground != -1:
            if self.measures.ground not in self.beacon_positions:
                self.beacon_positions[self.measures.ground] = self.curr_cell
                self.beacon_mapping[self.measures.ground] = self.curr_mapping
    
        if self.beacon in self.beacon_positions and (self.beacon - 1) in self.beacon_positions:
            self.beacon_path = self.path_finding(
                self.beacon_positions[self.beacon - 1],
                self.beacon_positions[self.beacon]
            )
            with open(self.pathfile, 'a+') as planfile:
                if self.beacon - 1 == 0:
                    start_cell = self.beacon_positions[self.beacon - 1]
                    planfile.write(f"{start_cell[0]} {start_cell[1]}")
                for idx, cell in enumerate(self.beacon_path):
                    line = f"\n{cell[0]} {cell[1]}"
                    if idx == len(self.beacon_path) - 1:
                        line += f" #{self.beacon}"
                    planfile.write(line)
            self.beacon += 1
    
        if self.beacon == int(self.nBeacons) and self.beaconCanWrite:
            self.beacon_path = self.path_finding(
                self.beacon_positions[self.beacon - 1],
                self.beacon_positions[0]
            )
            with open(self.pathfile, 'a') as planfile:
                for cell in self.beacon_path:
                    planfile.write(f"\n{cell[0]} {cell[1]}")
            self.beaconCanWrite = False
    
        if not self.tree_path:
            target_cell = self.newtowalk_cells[0] if self.newtowalk_cells else [0, 0]
            if self.curr_cell != target_cell:
                self.tree_path = self.path_finding(self.curr_cell, target_cell)
            else:
                return ("end", 0, cardinal)
    
        self.next_cell = self.tree_path.pop(0)
        delta_x = self.next_cell[0] - self.curr_cell[0]
        delta_y = self.next_cell[1] - self.curr_cell[1]
        delta = (delta_x, delta_y)
    
        direction_map = {
            "N": {
                (2, 0): ('walk', 0, "N"),
                (0, 2): ("rot_left", 90, "N"),
                (-2, 0): ("rot_right", 180, "N"),
                (0, -2): ("rot_right", 90, "N")
            },
            "S": {
                (-2, 0): ('walk', 0, "S"),
                (0, -2): ("rot_left", 90, "S"),
                (2, 0): ("rot_right", 180, "S"),
                (0, 2): ("rot_right", 90, "S")
            },
            "E": {
                (0, -2): ('walk', 0, "E"),
                (2, 0): ("rot_left", 90, "E"),
                (0, 2): ("rot_right", 180, "E"),
                (-2, 0): ("rot_right", 90, "E")
            },
            "O": {
                (0, 2): ('walk', 0, "O"),
                (-2, 0): ("rot_left", 90, "O"),
                (0, -2): ("rot_right", 180, "O"),
                (2, 0): ("rot_right", 90, "O")
            }
        }
    
        action = direction_map.get(cardinal, {}).get(delta)
        if action:
            return action
        else:
            return (state, 0, cardinal)
                
    def move(self, ki, km, kp, kd):
        error = ki - km
        p = kp * error
        d = kd * (error - self.errorPrev)
        self.errorPrev = error
        return p + d

    def move_one(self):
        cardinal, _,_,_,_ = self.DefineQuadrant()
        print("Move_one cardinal: " + cardinal)
        near = 1.65
        close = 1.7
        very_close = 1.85
        on_spot = 1.99
        ir_threshold = 2.3

        distance = abs(math.hypot(self.curr_gps[0] - self.curr_x, self.curr_gps[1] - self.curr_y))

        if (distance <= near or distance <= close or distance <= very_close or distance < on_spot ) and self.measures.irSensor[center] < ir_threshold:
            if distance <= near:
                velocity = 0.15
            elif distance <= close:
                velocity = 0.08
            elif distance <= very_close:
                velocity = 0.04
            elif distance < on_spot:
                velocity = 0.01
            self.perform_move(velocity)
            return "walk"
        else:
            if self.measures.irSensor[center] >= ir_threshold: 
                self.perform_move(0.0)
                def round_position(value):
                    int_value = int(abs(value))
                    if int_value % 2 != 0:
                        int_value += 1
                    return -int_value if value < 0 else int_value

                self.curr_x = round_position(self.curr_x)
                self.curr_y = round_position(self.curr_y)
                self.prev_x = self.curr_x
                self.prev_y = self.curr_y

            self.curr_gps = [self.curr_x, self.curr_y]
            delta = 2 if cardinal in ["N", "O"] else -2
            self.update_self_gps(delta, cardinal)
            return "choose"

    def perform_move(self, vel):
        print(f"Moving with velocity {vel}")
    
        def drive_and_calc(left_vel, right_vel):
            self.driveMotors(left_vel, right_vel)
            self.calc_pos(left_vel, right_vel)
    
        if self.measures.irSensor[left] >= 2.6:
            drive_and_calc(vel, vel - 0.0025)
        elif self.measures.irSensor[right] >= 2.6:
            drive_and_calc(vel - 0.0025, vel)
        else:
            compass = self.measures.compass
            if -10 <= compass <= 10:
                if compass < 0:
                    drive_and_calc(vel - 0.0025, vel)
                else:
                    drive_and_calc(vel, vel - 0.0025)
            elif 80 <= compass <= 100:
                if compass < 90:
                    drive_and_calc(vel - 0.0025, vel)
                else:
                    drive_and_calc(vel, vel - 0.0025)
            elif -100 <= compass <= -80:
                if compass < -90:
                    drive_and_calc(vel - 0.0025, vel)
                else:
                    drive_and_calc(vel, vel - 0.0025)
            else:
                drive_and_calc(vel, vel)

    def calc_pos(self, lpot, rpot):
        self.curr_lpout = (lpot + self.prev_lpout) / 2
        self.prev_lpout = self.curr_lpout
    
        self.curr_rpout = (rpot + self.prev_rpout) / 2
        self.prev_rpout = self.curr_rpout
    
        lin = (self.curr_lpout + self.curr_rpout) / 2
        rot = rpot - lpot
    
        self.prev_theta = math.radians(self.curr_theta) + rot
        self.curr_x = self.prev_x + lin * math.cos(self.prev_theta)
        self.curr_y = self.prev_y + lin * math.sin(self.prev_theta)
    
        self.prev_x, self.prev_y = self.curr_x, self.curr_y

    def Rotate_90_left(self, angel, cardinal):
        state = ""
        self.errorPrev = 0
        if cardinal == "N":
            if self.measures.compass >= 88 and self.measures.compass <= 92:
                self.driveMotors(0.0, 0.0)
                self.curr_theta = 90
                state = "walk"
            elif (75 <= self.measures.compass < 88):
                self.driveMotors(-0.01, 0.01)
                state = "rot_left"
            elif (60 < self.measures.compass < 75):
                self.driveMotors(-0.02, 0.02)
                state = "rot_left"
            else:
                self.driveMotors(-0.09, 0.09)
                state = "rot_left"
        if cardinal == "S":
            if self.measures.compass >= -92 and self.measures.compass <= -88:
                self.driveMotors(0.0, 0.0)
                self.curr_theta = -90
                state = "walk"
            elif (-100 <= self.measures.compass < -92):
                self.driveMotors(-0.01, 0.01)
                state = "rot_left"
            elif (-115 < self.measures.compass < -100):
                self.driveMotors(-0.02, 0.02)
                state = "rot_left"
            else:
                self.driveMotors(-0.09, 0.09)
                state = "rot_left"
        if cardinal == "O":
            if self.measures.compass >= 178 or self.measures.compass <= -178:
                self.driveMotors(0.0, 0.0)
                self.curr_theta = -180
                state = "walk"
            elif (165 <= self.measures.compass < 178):
                self.driveMotors(-0.01, 0.01)
                state = "rot_left"
            elif (155 < self.measures.compass < 165):
                self.driveMotors(-0.02, 0.02)
                state = "rot_left"
            else:
                self.driveMotors(-0.09, 0.09)
                state = "rot_left"
        if cardinal == "E":
            if self.measures.compass >= -2 and self.measures.compass <= 2:
                self.driveMotors(0.0, 0.0)
                self.curr_theta = 0
                state = "walk"
            elif (-10 <= self.measures.compass < -2):
                self.driveMotors(-0.01, 0.01)
                state = "rot_left"
            elif (-25 < self.measures.compass < -10):
                self.driveMotors(-0.02, 0.02)
                state = "rot_left"
            else:
                self.driveMotors(-0.09, 0.09)
                state = "rot_left"
        return state

    def Rotate_90_right(self, angle, cardinal):
        state = ""
        self.errorPrev = 0
        if angle == 180:
            if cardinal == "N":
                if self.measures.compass >= 178 or self.measures.compass <= -178:
                    self.driveMotors(-0.05, -0.05)
                    self.curr_theta = -180
                    state = "walk"
                elif (-178 < self.measures.compass <= -168):
                    self.driveMotors(0.01, -0.01)
                    state = "rot_right"
                elif (-168 < self.measures.compass < -155):
                    self.driveMotors(0.02, -0.02)
                    state = "rot_right"
                else:
                    self.driveMotors(0.09, -0.09)
                    state = "rot_right"
            if cardinal == "S":
                if self.measures.compass >= -2 and self.measures.compass <= 2:
                    self.driveMotors(0.0, 0.0)
                    self.curr_theta = 0
                    state = "walk"
                elif (2 < self.measures.compass <= 10):
                    self.driveMotors(0.01, -0.01)
                    state = "rot_right"
                elif (10 < self.measures.compass < 25):
                    self.driveMotors(0.02, -0.02)
                    state = "rot_right"
                else:
                    self.driveMotors(0.09, -0.09)
                    state = "rot_right"
            if cardinal == "O":
                if self.measures.compass >= -92 and self.measures.compass <= -88 :
                    self.driveMotors(0.0, 0.0)
                    self.curr_theta = -90
                    state = "walk"
                elif (-88 < self.measures.compass <= -75):
                    self.driveMotors(0.01, -0.01)
                    state = "rot_right"
                elif (-75 < self.measures.compass < -65):
                    self.driveMotors(0.02, -0.02)
                    state = "rot_right"
                else:
                    self.driveMotors(0.09, -0.09)
                    state = "rot_right"
            if cardinal == "E":
                if self.measures.compass >= 88 and self.measures.compass <= 92:
                    self.driveMotors(0.0, 0.0)
                    self.curr_theta = 90
                    state = "walk"
                elif (92 < self.measures.compass <= 100):
                    self.driveMotors(0.01, -0.01)
                    state = "rot_right"
                elif (100 < self.measures.compass < 115):
                    self.driveMotors(0.02, -0.02)
                    state = "rot_right"
                else:
                    self.driveMotors(0.09, -0.09)
                    state = "rot_right"
        else:
            if cardinal == "N":
                if self.measures.compass >= -92 and self.measures.compass <= -88:
                    self.driveMotors(0.0, 0.0)
                    self.curr_theta = -90
                    state = "walk"
                elif (-88 < self.measures.compass <= -75):
                    self.driveMotors(0.01, -0.01)
                    state = "rot_right"
                elif (-75 < self.measures.compass < -65):
                    self.driveMotors(0.02, -0.02)
                    state = "rot_right"
                else:
                    self.driveMotors(0.09, -0.09)
                    state = "rot_right"
            if cardinal == "S":
                if self.measures.compass >= 88 and self.measures.compass <= 92:
                    self.driveMotors(0.0, 0.0)
                    self.curr_theta = 90
                    state = "walk"
                elif (92 < self.measures.compass <= 100):
                    self.driveMotors(0.01, -0.01)
                    state = "rot_right"
                elif (100 < self.measures.compass < 115):
                    self.driveMotors(0.02, -0.02)
                    state = "rot_right"
                else:
                    self.driveMotors(0.09, -0.09)
                    state = "rot_right"
            if cardinal == "O":
                if self.measures.compass >= -2 and self.measures.compass <= 2:
                    self.driveMotors(0.0, 0.0)
                    self.curr_theta = 0
                    state = "walk"
                elif (2 < self.measures.compass <= 10):
                    self.driveMotors(0.01, -0.01)
                    state = "rot_right"
                elif (10 < self.measures.compass < 25):
                    self.driveMotors(0.02, -0.02)
                    state = "rot_right"
                else:
                    self.driveMotors(0.09, -0.09)
                    state = "rot_right"
            if cardinal == "E":
                if self.measures.compass >= 178 or self.measures.compass <= -178:
                    self.driveMotors(0.0, 0.0)
                    self.curr_theta = -180
                    state = "walk"
                elif (-178 < self.measures.compass <= -168):
                    self.driveMotors(0.01, -0.01)
                    state = "rot_right"
                elif (-168 < self.measures.compass < -155):
                    self.driveMotors(0.02, -0.02)
                    state = "rot_right"
                else:
                    self.driveMotors(0.09, -0.09)
                    state = "rot_right"

        return state

    def write_planning(self):
        if len(self.beacon_positions) >= int(self.nBeacons):
            print("All beacons found. Computing best path...")
            MAP = self.map
            MAP_x_initial = self.init_mapping[1]
            MAP_y_initial = self.init_mapping[0]
            list_beacons_coords = [
                (self.beacon_mapping[beacon][0], self.beacon_mapping[beacon][1], beacon)
                for beacon in sorted(self.beacon_positions.keys())
            ]

            best_path = self.BestPath(MAP, MAP_x_initial, MAP_y_initial, list_beacons_coords)
            
            with open(self.pathfile, 'w') as planfile:
                for idx, cell in enumerate(best_path):
                    x, y = cell
                    if idx == 0:
                        planfile.write(f"{x} {y}")
                    else:
                        planfile.write(f"\n{x} {y}")
                    for beacon in self.beacon_positions:
                        beacon_x, beacon_y = self.beacon_mapping[beacon]
                        if x == beacon_x and y == beacon_y:
                            planfile.write(f" #{beacon}")

    def BestPath(self, MAP, MAP_x_initial, MAP_y_initial, list_beacons_coords):
        def symbol_to_code(symbol):
            if symbol == ' ':
                return 0   # Empty space
            elif symbol == '|':
                return 70  # Wall vertical
            elif symbol == '-':
                return 70  # Wall horizontal
            elif symbol == 'X':
                return 60  # Open path
            elif symbol == '0':
                return 80  # Starting position
            elif symbol.isdigit():
                return symbol  # Beacons
            else:
                return 0   # Default to empty space
    
        # Convert the map to a numeric NumPy array
        # map_numpy = np.array([[symbol_to_code(cell) for cell in row] for row in MAP])
        map_numpy = np.array(MAP)
        # Include the initial position with a beacon ID
        initial_position = (MAP_y_initial, MAP_x_initial, 0)
        list_beacons_coords.insert(0, initial_position)
    
        # Sort beacons by their IDs
        beacons_sorted = sorted(list_beacons_coords, key=lambda x: x[2])
    
        # Prepare the list of positions to visit without the beacon ID
        positions_to_visit = [(y, x) for y, x, _ in beacons_sorted]
    
        complete_path = []
        current_position = (MAP_y_initial, MAP_x_initial)
    
        for checkpoint in positions_to_visit[1:]:
            segment_path = find_path_segment(map_numpy, current_position, checkpoint[:2])
            if segment_path is None:
                print(f"No path found between {current_position} and {checkpoint}")
                continue
            complete_path.extend(segment_path)
            current_position = checkpoint[:2]
    
        best_path = [(x, y) for y, x in complete_path]
    
        return best_path
    
def find_path_segment(map_numpy, start, end):
    if start == end:
        return []

    start_y, start_x = start
    end_y, end_x = end

    map_for_path = np.zeros_like(map_numpy, dtype=int)
    map_for_path[start_y, start_x] = 1

    # Pathfinding loop
    while map_for_path[end_y, end_x] == 0:
        max_value = np.amax(map_for_path)
        positions = np.argwhere(map_for_path == max_value)
        for pos in positions:
            j, i = pos
            if map_numpy[j, i] != 70:
                for dj, di in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
                    new_j, new_i = j + dj, i + di
                    if 0 <= new_j < map_numpy.shape[0] and 0 <= new_i < map_numpy.shape[1]:
                        if map_for_path[new_j, new_i] == 0 and map_numpy[new_j, new_i] != 70:
                            map_for_path[new_j, new_i] = max_value + 1
        if max_value == np.amax(map_for_path):
            print("No path found")
            return None  # No path found

    # Backtracking to find the path
    path = []
    current_y, current_x = end_y, end_x
    max_value = map_for_path[end_y, end_x]
    while max_value > 1:
        path.append((current_y, current_x))
        max_value -= 1
        for dj, di in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
            new_j, new_i = current_y + dj, current_x + di
            if 0 <= new_j < map_for_path.shape[0] and 0 <= new_i < map_for_path.shape[1]:
                if map_for_path[new_j, new_i] == max_value:
                    current_y, current_x = new_j, new_i
                    break

    path.append((start_y, start_x))
    return path[::-1]

class Map():
    def __init__(self, filename):
        tree = ET.parse(filename)
        root = tree.getroot()
        
        self.labMap = [[' '] * (CELL_COLS*2-1) for i in range(CELL_ROWS*2-1) ]
        i=1
        for child in root.iter('Row'):
           line=child.attrib['Pattern']
           row =int(child.attrib['Pos'])
           if row % 2 == 0:
               for c in range(len(line)):
                   if (c+1) % 3 == 0:
                       if line[c] == '|':
                           self.labMap[row][(c+1)//3*2-1]='|'
                       else:
                           None
           else:
               for c in range(len(line)):
                   if c % 3 == 0:
                       if line[c] == '-':
                           self.labMap[row][c//3*2]='-'
                       else:
                           None
               
           i=i+1


rob_name = "pClient4"
host = "localhost"
pos = 1
mapc = None
mapfile = "myrob.map" 
pathfile = "myrob.path"

for i in range(1, len(sys.argv),2):
    if (sys.argv[i] == "--host" or sys.argv[i] == "-h") and i != len(sys.argv) - 1:
        host = sys.argv[i + 1]
    elif (sys.argv[i] == "--pos" or sys.argv[i] == "-p") and i != len(sys.argv) - 1:
        pos = int(sys.argv[i + 1])
    elif (sys.argv[i] == "--robname" or sys.argv[i] == "-r") and i != len(sys.argv) - 1:
        rob_name = sys.argv[i + 1]
    elif (sys.argv[i] == "--map" or sys.argv[i] == "-m") and i != len(sys.argv) - 1:
        mapc = Map(sys.argv[i + 1])
    elif (sys.argv[i] == "--file" or sys.argv[i] == "-f") and i != len(sys.argv) - 1:
        mapfile = sys.argv[i + 1] + ".map"; pathfile = sys.argv[i + 1] + ".path"

    else:
        print("Unkown argument", sys.argv[i])
        quit()

if __name__ == '__main__':
    rob=MyRob(rob_name,pos,[0.0,90.0,-90.0,180.0],host,mapfile,pathfile)

    if mapc != None:
        rob.setMap(mapc.labMap)
        rob.printMap()
    
    rob.run()