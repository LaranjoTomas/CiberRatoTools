# Robotic challenge solver using the CiberRato simulation environment

 Development of a robotic agent to command a simulated mobile robot in order to implement a set of robotic tasks, involving different navigation skills.

### Objectives

1. Localization: The agent needs to navigate and localize itself in an unknown maze, using the movement model, the distance to obstacles and the compass. GPS is not available. Motors, distance sensors and compass are noisy. You can consult the C4-config.xml file to get the noise parameters. Collisions with walls will be penalized.

2. Mapping: The agent needs to explore an unknown maze in order to completely map its navigable cells. At the same time, the agent needs to localize target spots placed in the maze. The number of target spots may change between mazes. After completing the mapping task, the agent should return to the starting spot. Collisions with walls will be penalized.

3. Planning: The agent needs to compute a closed path with minimal cost that allows to visit all target spots, starting and ending in the starting spot.

These objectives were achieved using S.L.A.M. techniques. 

![Alt Text](https://media0.giphy.com/media/x4bIza5QxRQopRjURd/giphy.gif)


### Dependencies

libqt4-dev

### Building the Simulator

```
make
```

### Running the agent (for the Challenge 4)

```
./startC4
cd agent
./run.sh
```