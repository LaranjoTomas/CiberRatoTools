import sys
import numpy as np
from croblink import *
from math import *
import xml.etree.ElementTree as ET

CELLROWS=7
CELLCOLS=14

class MyRob(CRobLinkAngs):
    def __init__(self, rob_name, rob_id, angles, host):
        CRobLinkAngs.__init__(self, rob_name, rob_id, angles, host)
        self.lap_time = 0
        self.readSensors()

        global MAP
        global MAP_x_current
        global MAP_y_current
        global MAP_x_initial
        global MAP_y_initial

        MAP = []
        for i in range(27):
            rows = 55 * [10]
            MAP.append(rows)

        MAP_x_current = MAP_y_current = []

        global GPS_x_initial
        global GPS_y_initial
        global GPS_x_start
        global GPS_y_start
        global GPS_x_current
        global GPS_y_current
        GPS_x_initial = self.measures.x
        GPS_y_initial = self.measures.y
        GPS_x_start = self.measures.x
        GPS_y_start = self.measures.y


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
        stopped_state = 'run'

        while True:
            self.readSensors()

            self.measures.gpsReady = True
            self.measures.gpsDirReady = True

            if self.measures.endLed:
                print(self.robName + " exiting")
                quit()

            if state == 'stop' and self.measures.start:
                state = stopped_state

            if state != 'stop' and self.measures.stop:
                stopped_state = state
                state = 'stop'

            if state == 'run':
                if self.measures.visitingLed==True:
                    state='wait'
                if self.measures.ground==0:
                    self.setVisitingLed(True);
                self.wander()
            elif state=='wait':
                self.setReturningLed(True)
                if self.measures.visitingLed==True:
                    self.setVisitingLed(False)
                if self.measures.returningLed==True:
                    state='return'
                self.driveMotors(0.0,0.0)
            elif state=='return':
                if self.measures.visitingLed==True:
                    self.setVisitingLed(False)
                if self.measures.returningLed==True:
                    self.setReturningLed(False)
                self.wander()

    def DefineQuadrant(self):
        self.readSensors()
        compass = self.measures.compass

        Quadrant = [False,False,False,False]

        if abs(compass) <= 45:
            Quadrant[0] = True
        elif compass > 45 and compass <= 135:
            Quadrant[1] = True
        elif abs(compass) >= 135:
            Quadrant[2] = True
        elif compass <= -45 and compass >= -135:
            Quadrant[3] = True

        return Quadrant

    def Mapper(self, flags, y, x):
        CENTER = 0
        LEFT = 1
        RIGHT = 2
        threshold = 1.15

        self.readSensors()

        center_sensor = self.measures.irSensor[CENTER]
        left_sensor = self.measures.irSensor[LEFT]
        right_sensor = self.measures.irSensor[RIGHT]

        def update_map(sensor_value, threshold, map_coords, wall_value, empty_value, explore_value):
            if sensor_value >= threshold:
                MAP[map_coords[0]][map_coords[1]] = wall_value
            else:
                MAP[map_coords[0]][map_coords[1]] = empty_value
                if MAP[map_coords[2]][map_coords[3]] != 80:
                    MAP[map_coords[2]][map_coords[3]] = explore_value

        if flags[0]:
            update_map(center_sensor, threshold, (y, x + 1, y, x + 2), 30, 20, 60)
            update_map(left_sensor, threshold, (y - 1, x, y - 2, x), 40, 20, 60)
            update_map(right_sensor, threshold, (y + 1, x, y + 2, x), 40, 20, 60)

            ahead, ahead2 = MAP[y][x + 1], MAP[y][x + 2]
            right, right2 = MAP[y + 1][x], MAP[y + 2][x]
            left, left2 = MAP[y - 1][x], MAP[y - 2][x]

        elif flags[1]:
            update_map(center_sensor, threshold, (y - 1, x, y - 2, x), 40, 20, 60)
            update_map(left_sensor, threshold, (y, x - 1, y, x - 2), 30, 20, 60)
            update_map(right_sensor, threshold, (y, x + 1, y, x + 2), 30, 20, 60)

            ahead, ahead2 = MAP[y - 1][x], MAP[y - 2][x]
            right, right2 = MAP[y][x + 1], MAP[y][x + 2]
            left, left2 = MAP[y][x - 1], MAP[y][x - 2]

        elif flags[2]:
            update_map(center_sensor, threshold, (y, x - 1, y, x - 2), 30, 20, 60)
            update_map(left_sensor, threshold, (y + 1, x, y + 2, x), 40, 20, 60)
            update_map(right_sensor, threshold, (y - 1, x, y - 2, x), 40, 20, 60)

            ahead, ahead2 = MAP[y][x - 1], MAP[y][x - 2]
            right, right2 = MAP[y - 1][x], MAP[y - 2][x]
            left, left2 = MAP[y + 1][x], MAP[y + 2][x]

        elif flags[3]:
            update_map(center_sensor, threshold, (y + 1, x, y + 2, x), 40, 20, 60)
            update_map(left_sensor, threshold, (y, x + 1, y, x + 2), 30, 20, 60)
            update_map(right_sensor, threshold, (y, x - 1, y, x - 2), 30, 20, 60)

            ahead, ahead2 = MAP[y + 1][x], MAP[y + 2][x]
            right, right2 = MAP[y][x - 1], MAP[y][x - 2]
            left, left2 = MAP[y][x + 1], MAP[y][x + 2]

        return ahead, ahead2, right, right2, left, left2    

    def Move(self, Z):
        self.readSensors()
    
        GPS_x = self.measures.x - GPS_x_initial
        GPS_y = self.measures.y - GPS_y_initial
    
        # Define the linear setpoint (exact coords for the next cell given by pair numbers)
        GPS_x_current_vet = [i for i in range(-26, 28, 2)]
        GPS_x_current = GPS_x_current_vet[next_cell_to_explore(GPS_x_current_vet, GPS_x)]
    
        GPS_y_current_vet = [j for j in range(-12, 14, 2)]
        GPS_y_current = GPS_y_current_vet[next_cell_to_explore(GPS_y_current_vet, GPS_y)]
    
        error_x = error_y = 100
        error_x_last = error_y_last = 0
    
        lin = 0.115
        kp = 0.01
        kd = 0.1
        threshold = 0.225
    
        while (Z[0] and error_x > threshold) or (Z[1] and error_y > threshold) or (Z[2] and error_x > threshold) or (Z[3] and error_y > threshold):
            self.readSensors()
            GPS_x = self.measures.x - GPS_x_initial
            GPS_y = self.measures.y - GPS_y_initial
    
            if Z[0]:
                error_x = (GPS_x_current + 2) - GPS_x
                error_y = GPS_y_current - GPS_y
                rot = error_y * kp + (error_y - error_y_last) / 2 * kd
                right_rotation = lin + rot
                left_rotation = lin - rot
                self.driveMotors(left_rotation, right_rotation)
                error_y_last = error_y
    
            if Z[1]:
                error_x = GPS_x_current - GPS_x
                error_y = (GPS_y_current + 2) - GPS_y
                rot = error_x * kp + (error_x - error_x_last) / 2 * kd
                right_rotation = lin - rot
                left_rotation = lin + rot
                self.driveMotors(left_rotation, right_rotation)
                error_x_last = error_x
    
            if Z[2]:
                error_x = GPS_x - (GPS_x_current - 2)
                error_y = GPS_y - GPS_y_current
                rot = error_y * kp + (error_y - error_y_last) / 2 * kd
                right_rotation = lin + rot
                left_rotation = lin - rot
                self.driveMotors(left_rotation, right_rotation)
                error_y_last = error_y
    
            if Z[3]:
                error_x = GPS_x - GPS_x_current
                error_y = GPS_y - (GPS_y_current - 2)
                rot = error_x * kp + (error_x - error_x_last) / 2 * kd
                right_rotation = lin - rot
                left_rotation = lin + rot
                self.driveMotors(left_rotation, right_rotation)
                error_x_last = error_x
                
    def Rotate_90_Left(self):
        def get_true_compass(compass):
            compass_vector = [0, 90, -180, -90, 180]
            true_compass = compass_vector[next_cell_to_explore(compass_vector, compass)]
            return -180 if true_compass == 180 else true_compass
    
        def calculate_rotation_error(current_compass, target_compass):
            rotation_error = target_compass - current_compass
            if rotation_error > 120:
                rotation_error -= 360
            return rotation_error
    
        def drive_with_rotation_error(rotation_error):
            Kd_angulo = 0.005
            rotation = Kd_angulo * rotation_error
            right_motor_speed = rotation
            left_motor_speed = -rotation
            self.driveMotors(left_motor_speed, right_motor_speed)
    
        target_compass = get_true_compass(self.measures.compass) + 90
        rotation_error = 100
    
        while abs(rotation_error) >= 1:
            self.readSensors()
            current_compass = self.measures.compass
            rotation_error = calculate_rotation_error(current_compass, target_compass)
            drive_with_rotation_error(rotation_error)

    def Rotate_90_Right(self):
        def get_true_compass(compass):
            compass_vector = [0, 90, -180, -90, 180]
            true_compass = compass_vector[next_cell_to_explore(compass_vector, compass)]
            return -180 if true_compass == 180 else true_compass
    
        def calculate_rotation_error(current_compass, target_compass):
            rotation_error = target_compass - current_compass
            if rotation_error < -120:
                rotation_error += 360
            return rotation_error
    
        def drive_with_rotation_error(rotation_error):
            Kd_angulo = 0.005
            rotation = Kd_angulo * rotation_error
            right_motor_speed = rotation
            left_motor_speed = -rotation
            self.driveMotors(left_motor_speed, right_motor_speed)
    
        target_compass = get_true_compass(self.measures.compass) - 90
        rotation_error = 100
    
        while abs(rotation_error) >= 1:
            self.readSensors()
            current_compass = self.measures.compass
            rotation_error = calculate_rotation_error(current_compass, target_compass)
            drive_with_rotation_error(rotation_error)

    def path_finding(self, map_numpy, not_visited_x, not_visited_y, current_x, current_y):
        try:
            # Combine x and y coordinates of not visited positions into a list of tuples
            not_visited_positions = list(zip(not_visited_y, not_visited_x))
            print("List of not visited positions:", not_visited_positions)
    
            linear_movements = []
            overall_movements = []
    
            # Dijkstra's Algorithm
            for target_y, target_x in not_visited_positions:
                map_for_path = np.zeros_like(map_numpy)
                map_for_path[current_y, current_x] = 1
                print("Initial map_for_path:\n", map_for_path)
    
                while map_for_path[target_y, target_x] == 0:
                    max_value = np.amax(map_for_path)
                    possible_positions = np.argwhere(map_for_path == max_value)
                    print("Possible positions to go (j, i):", possible_positions)
    
                    for j, i in possible_positions:
                        if map_numpy[j, i] in [20, 80, 90]:
                            for dj, di in [(1, 0), (-1, 0), (0, 1), (0, -1)]:
                                if map_for_path[j + dj, i + di] == 0 and map_numpy[j + dj, i + di] in [20, 60, 80]:
                                    map_for_path[j + dj, i + di] = max_value + 1
                    print("Updated map_for_path:\n", map_for_path)
    
                movements = []
                max_value = np.amax(map_for_path)
                current_y, current_x = target_y, target_x
    
                for _ in range(max_value - 1):
                    sides_array = np.array([[0, map_for_path[current_y - 1, current_x], 0],
                                            [map_for_path[current_y, current_x - 1], map_for_path[current_y, current_x], map_for_path[current_y, current_x + 1]],
                                            [0, map_for_path[current_y + 1, current_x], 0]])
    
                    j_i, i_i = np.where(sides_array == max_value - 1)
                    print("Sides array:\n", sides_array)
                    print("Positions in sides array (j_i, i_i):", j_i, i_i)
    
                    if len(j_i) == 0 or len(i_i) == 0:
                        break  # No valid moves found, break the loop
    
                    ji = (j_i[0], i_i[0])
    
                    if ji == (0, 1):
                        movements.append('DOWN')
                        current_y -= 1
                    elif ji == (1, 0):
                        movements.append('RIGHT')
                        current_x -= 1
                    elif ji == (1, 2):
                        movements.append('LEFT')
                        current_x += 1
                    elif ji == (2, 1):
                        movements.append('UP')
                        current_y += 1
    
                    max_value -= 1
    
                movements = movements[1::2][::-1]
                overall_movements.append(movements)
                print('The agent must follow the next movements: ' + str(movements))
    
                num_rotations = sum(1 for i in range(1, len(movements)) if movements[i] != movements[i - 1])
                real_num_movements = num_rotations + len(movements)
    
                linear_movements.append(real_num_movements)
    
            print(linear_movements)
            print(overall_movements)
            print('The closest path is = ' + str(overall_movements[linear_movements.index(min(linear_movements))]))
            return overall_movements[linear_movements.index(min(linear_movements))]
        except:
            print('Maze has been explored. Goodbye!')
            exit()
            
    def Pather(self, next_movements):
        def rotate_until_facing(quadrant, direction_index):
            while not quadrant[direction_index]:
                self.Rotate_90_Left()
                quadrant = self.DefineQuadrant()
            return quadrant
    
        def move_and_update_quadrant(quadrant):
            self.Move(quadrant)
            return self.DefineQuadrant()
    
        def get_rotation_function(prev_direction, next_direction):
            rotation_map = {
                ('LEFT', 'DOWN'): self.Rotate_90_Left,
                ('LEFT', 'UP'): self.Rotate_90_Right,
                ('RIGHT', 'DOWN'): self.Rotate_90_Right,
                ('RIGHT', 'UP'): self.Rotate_90_Left,
                ('UP', 'LEFT'): self.Rotate_90_Left,
                ('UP', 'RIGHT'): self.Rotate_90_Right,
                ('DOWN', 'LEFT'): self.Rotate_90_Right,
                ('DOWN', 'RIGHT'): self.Rotate_90_Left
            }
            return rotation_map.get((prev_direction, next_direction))
    
        quadrant = self.DefineQuadrant()
        direction_map = {
            'LEFT': 2,
            'RIGHT': 0,
            'UP': 1,
            'DOWN': 3
        }
    
        if next_movements[0] in direction_map:
            quadrant = rotate_until_facing(quadrant, direction_map[next_movements[0]])
    
        quadrant = move_and_update_quadrant(quadrant)
        print(next_movements)
    
        for i in range(1, len(next_movements)):
            if next_movements[i] == next_movements[i - 1]:
                quadrant = move_and_update_quadrant(quadrant)
            else:
                rotation_function = get_rotation_function(next_movements[i - 1], next_movements[i])
                if rotation_function:
                    rotation_function()
                    quadrant = self.DefineQuadrant()
                    quadrant = move_and_update_quadrant(quadrant)

    def save_map(self, MAP, Map_x_initial, Map_y_initial):
        def cell_to_char(cell):
            cell_char_map = {
                80: 'X',
                90: 'X',
                20: 'X',
                60: 'X',
                30: '|',
                40: '-',
                10: ' ',
                50: 'I'
            }
            return cell_char_map.get(cell, ' ')
    
        MAP[Map_y_initial][Map_x_initial] = 50
    
        # Convert the map to a string representation
        MAP_str = "\n".join("".join(cell_to_char(cell) for cell in row) for row in MAP)
    
        print("Final map to be written:\n", MAP_str)  # Debugging print statement
    
        # Write the map to a file
        with open('mymap.txt', 'w') as f:
            f.write(MAP_str)

    def wander(self):
        # Start VARIABLES
        global GPS_x_initial, GPS_y_initial, GPS_x_start, GPS_y_start
        global GPS_x_current, GPS_y_current, MAP
        global MAP_x_current, MAP_y_current, MAP_x_initial, MAP_y_initial
    
        self.readSensors()
    
        compass = self.measures.compass
    
        GPS_x = self.measures.x - GPS_x_initial
        GPS_y = self.measures.y - GPS_y_initial
    
        GPS_x_current = self.get_current_position(GPS_x, -26, 28, 2)
        GPS_y_current = self.get_current_position(GPS_y, -12, 14, 2)
    
        # Initial position in MAP
        MAP_x_initial, MAP_y_initial = 27, 13
    
        MAP_x_current = MAP_x_initial + GPS_x_current
        MAP_y_current = MAP_y_initial - GPS_y_current
    
        MAP_x_current = MAP_x_current if MAP_x_current else MAP_x_initial
        MAP_y_current = MAP_y_current if MAP_y_current else MAP_y_initial
    
        # Define zone
        Quadrant = self.DefineQuadrant()
    
        # Update map
        MAP[MAP_y_initial][MAP_x_initial] = 80
        F, F2, R, R2, L, L2 = self.Mapper(Quadrant, MAP_y_current, MAP_x_current)
    
        # Map analysis
        MAP[MAP_y_current][MAP_x_current] = 90
        MAP_Numpy = np.array(MAP)
        list_not_visited_y, list_not_visited_x = np.where(MAP_Numpy == 60)
        current_position_y, current_position_x = np.where(MAP_Numpy == 90)
    
        print("list_not_visited_y:", list_not_visited_y)
        print("list_not_visited_x:", list_not_visited_x)
        print("current_position_y:", current_position_y)
        print("current_position_x:", current_position_x)
    
        MAP[MAP_y_current][MAP_x_current] = 80
    
        if self.should_Rotate_90_Right(R2, R, F2):
            self.Rotate_90_Right()
        elif self.should_Rotate_90_Left(L2, L, F2):
            self.Rotate_90_Left()
        elif F == 20 and F2 != 80:  # Check if the cell ahead is empty and not visited
            self.Move(Quadrant)
        else:
            if not list_not_visited_y.size or not list_not_visited_x.size:
                print("No more cells to explore. Exiting.")
                self.save_map(MAP, MAP_x_initial, MAP_y_initial)
                exit()
    
            # Find the closest unexplored cell
            list_movements = self.path_finding(MAP_Numpy, list_not_visited_x, list_not_visited_y, current_position_x, current_position_y)
            print(list_movements)
            self.Pather(list_movements)
    
        self.driveMotors(0, 0)
    
        # Write MAP on txt
        self.save_map(MAP, MAP_x_initial, MAP_y_initial)

    def get_current_position(self, GPS, start, end, step):
        GPS_current_vet = [i for i in range(start, end, step)]
        return GPS_current_vet[next_cell_to_explore(GPS_current_vet, GPS)]

    def should_Rotate_90_Right(self, R2, R, F2):
        return R2 == 60 and R == 20 and F2 != 60

    def should_Rotate_90_Left(self, L2, L, F2):
        return L2 == 60 and L == 20 and F2 != 60

def next_cell_to_explore(list, N):
    pls_send_help = []
    for i in list:
        pls_send_help.append(abs(N - i))
    return pls_send_help.index(min(pls_send_help))

class Map():
    def __init__(self, filename):
        tree = ET.parse(filename)
        root = tree.getroot()
        
        self.labMap = [[' '] * (CELLCOLS*2-1) for i in range(CELLROWS*2-1) ]
        i=1
        for child in root.iter('Row'):
           line=child.attrib['Pattern']
           row =int(child.attrib['Pos'])
           if row % 2 == 0:  # this line defines vertical lines
               for c in range(len(line)):
                   if (c+1) % 3 == 0:
                       if line[c] == '|':
                           self.labMap[row][(c+1)//3*2-1]='|'
                       else:
                           None
           else:  # this line defines horijontal lines
               for c in range(len(line)):
                   if c % 3 == 0:
                       if line[c] == '-':
                           self.labMap[row][c//3*2]='-'
                       else:
                           None
               
           i=i+1


rob_name = "pClient1"
host = "localhost"
pos = 1
mapc = None

for i in range(1, len(sys.argv),2):
    if (sys.argv[i] == "--host" or sys.argv[i] == "-h") and i != len(sys.argv) - 1:
        host = sys.argv[i + 1]
    elif (sys.argv[i] == "--pos" or sys.argv[i] == "-p") and i != len(sys.argv) - 1:
        pos = int(sys.argv[i + 1])
    elif (sys.argv[i] == "--robname" or sys.argv[i] == "-r") and i != len(sys.argv) - 1:
        rob_name = sys.argv[i + 1]
    elif (sys.argv[i] == "--map" or sys.argv[i] == "-m") and i != len(sys.argv) - 1:
        mapc = Map(sys.argv[i + 1])
    else:
        print("Unkown argument", sys.argv[i])
        quit()

if __name__ == '__main__':
    rob=MyRob(rob_name,pos,[0.0,90.0,-90.0,180.0],host)

    if mapc != None:
        rob.setMap(mapc.labMap)
        rob.printMap()
    
    rob.run()