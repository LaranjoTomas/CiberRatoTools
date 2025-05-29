import sys
from croblink import CRobLinkAngs
import xml.etree.ElementTree as ET

from controller_constants import controller, ControllerType

set_point = 1.0
feedback = 0.5

CELLROWS = 7
CELLCOLS = 14

class MyRob(CRobLinkAngs):
    def __init__(self, rob_name, rob_id, angles, host):
        super().__init__(rob_name, rob_id, angles, host)

    def set_map(self, lab_map):
        self.lab_map = lab_map

    def print_map(self):
        for row in reversed(self.lab_map):
            print(''.join(row))

    def run(self):
        if self.status != 0:
            print("Connection refused or error")
            quit()

        state = 'stop'
        while True:
            self.readSensors()

            if self.measures.endLed:
                print(f"{self.rob_name} exiting. Simulation time: {self.measures.time}")
                quit()

            if state == 'stop' and self.measures.start:
                state = 'run'

            if state != 'stop' and self.measures.stop:
                state = 'stop'

            if state == 'run':
                if self.measures.visitingLed:
                    state = 'wait'
                if self.measures.ground == 0:
                    self.setVisitingLed(True)
                self.wander()
            elif state == 'wait':
                self.setReturningLed(True)
                if self.measures.visitingLed:
                    self.setVisitingLed(False)
                if self.measures.returningLed:
                    state = 'return'
                self.driveMotors(0.0, 0.0)
            elif state == 'return':
                if self.measures.visitingLed:
                    self.setVisitingLed(False)
                if self.measures.returningLed:
                    self.setReturningLed(False)
                self.wander()

    def wander(self):
        center_id = 0
        left_id = 1
        right_id = 2
    
        # Thresholds for detecting walls
        too_close_threshold = 7.5
        close_threshold = 1.1
    
        center_sensor = self.measures.irSensor[center_id]
        right_sensor = self.measures.irSensor[right_id]
        left_sensor = self.measures.irSensor[left_id]
    
        forward_speed, rotation_speed = self.determine_movement(center_sensor, right_sensor, left_sensor, close_threshold)
        self.driveMotors(forward_speed + rotation_speed, forward_speed - rotation_speed)
    
        self.handle_too_close_sensors(right_sensor, left_sensor, too_close_threshold)
    
    def determine_movement(self, center_sensor, right_sensor, left_sensor, close_threshold):
        forward_speed = 0
        rotation_speed = 0
    
        if center_sensor <= close_threshold:
            forward_speed = 0.15
            print("going forward")
        elif right_sensor < left_sensor:
            rotation_speed = 0.15
            print("Rotate right based on closer right sensor")
        else:
            rotation_speed = -0.15
            print("Rotate left based on closer left sensor")
    
        return forward_speed, rotation_speed
    
    def handle_too_close_sensors(self, right_sensor, left_sensor, too_close_threshold):
        if right_sensor >= too_close_threshold:
            print("Rotating left")
            self.driveMotors(-0.15, 0.15)
    
        if left_sensor >= too_close_threshold:
            print("Rotating right")
            self.driveMotors(0.15, -0.15)

class Map:
    def __init__(self, filename):
        self.lab_map = self._load_map(filename)

    def _load_map(self, filename):
        tree = ET.parse(filename)
        root = tree.getroot()
        lab_map = [[' '] * (CELLCOLS * 2 - 1) for _ in range(CELLROWS * 2 - 1)]

        for child in root.iter('Row'):
            line = child.attrib['Pattern']
            row = int(child.attrib['Pos'])
            if row % 2 == 0:  # Vertical lines
                for c in range(len(line)):
                    if (c + 1) % 3 == 0:
                        lab_map[row][(c + 1) // 3 * 2 - 1] = '|' if line[c] == '|' else ' '
            else:  # Horizontal lines
                for c in range(len(line)):
                    if c % 3 == 0:
                        lab_map[row][c // 3 * 2] = '-' if line[c] == '-' else ' '

        return lab_map

def parse_arguments():
    host = "localhost"
    pos = 1
    rob_name = "pClient1"
    mapc = None

    for i in range(1, len(sys.argv), 2):
        if (sys.argv[i] in ["--host", "-h"]) and i != len(sys.argv) - 1:
            host = sys.argv[i + 1]
        elif (sys.argv[i] in ["--pos", "-p"]) and i != len(sys.argv) - 1:
            pos = int(sys.argv[i + 1])
        elif (sys.argv[i] in ["--robname", "-r"]) and i != len(sys.argv) - 1:
            rob_name = sys.argv[i + 1]
        elif (sys.argv[i] in ["--map", "-m"]) and i != len(sys.argv) - 1:
            mapc = Map(sys.argv[i + 1])
        else:
            print(f"Unknown argument {sys.argv[i]}")
            quit()

    return rob_name, pos, host, mapc

if __name__ == '__main__':

    lap_time_vet = [0]
    lap = []

    rob_name, pos, host, mapc = parse_arguments()
    rob = MyRob(rob_name, pos, [0.0, 60.0, -60.0, 180.0], host)
    if mapc is not None:
        rob.set_map(mapc.lab_map)
        rob.print_map()
    
    rob.run()