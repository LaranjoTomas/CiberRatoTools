# CiberRatoTools

## Overview
CiberRatoTools is a robot simulation environment developed by the University of Aveiro for the Micro-Rato competition (Ciber-Rato modality). The system simulates robots navigating through a labyrinth, with the objective of reaching beacon areas and returning to their starting positions.

## Project Structure

- **simulator/:** Core simulation engine that manages the virtual environment
- **Viewer/:** Graphical interface that displays the simulation in real-time
- **pClient/:** Python client for robot control
- **jClient/:** Java client interface
- **robsample/:** Sample robot implementation
- **libRobSock/:** Communication library
- **logplayer/:** Tool for replaying simulation logs
- **GUISample/:** Sample GUI implementation


## pClient
The pClient folder contains Python implementations for controlling robots in the simulation. This is where you can develop your robot's behavior and decision-making algorithms.

Using pClient
To use the Python client:

1. Make sure the simulator is running
2. Run your Python robot script with appropriate parameters, for example:
```bash
python yourRobotScript.py -h localhost -p 12345
```

## Developing Robot Logic
The Python client allows you to:

1. Read sensor data (IR sensors, beacon sensors, GPS)
2. Control robot motors
3. Process map information
4. Implement navigation strategies

Communication uses XML messages for registration, sensor data, and motor commands.

## Building and Running

Building the Project
To build the entire project:

```bash
cd ciberRatoTools
make
```

Starting the Simulation
Several scripts are provided to start different components, navigate to the folder pClient and run the .bash scripts

## License
This project is licensed under the GNU General Public License v2.0. See the COPYING file for details.

## Authors
- **Tomás Laranjo, 107542**
- **Daniel Pedrinho, 107378**