# ENPM 661 Project-3 Phase2

## Team Members
### Pulkit Mehta (UID: 117551693)
### Anirudh Krishnan Komaralingam (UID: 117446432)


## INSTRUCTIONS TO RUN

Download/Clone the package into the workspace and build using catkin build or catkin_make.
Source the workspace.

### 2-D Path Planning

To run A* path planning algorithm run the below from src folder the workspace:

```bash
$ cd proj3_phase2/scripts/
$ python3 Phase2.py --Start="[1,1,30]" --End="[9,9,0]" --RPM="[10,15]"
```

- A start location with x-coordinate as 1, y-cooradinate as 1 and orientation as 30 degrees can be specified by the "--Start" argument as: --Start="[1,1,30]"
- Similarly the goal location can be specified using the "--End" argument.
- The "--RPM" argument can be used to specify both the RPM values.

![](https://github.com/pulkitmehta09/ENPM661-Project3/blob/main/proj3_phase2/Simulation%20Videos/output.gif)

### Gazebo Simulation

Make the node as executable by running the following from src folder in the workspace:
```bash
$ cd proj3_phase2/scripts/
$ chmod +x node.py
```

To run the simulation:

```bash
roslaunch proj3_phase2 proj3phase2.launch start:="[5,1,30]" end:="[3,7,0]" RPM:="[10,10]" clearance:="0.1"
```

- A start location with x-coordinate as 5, y-cooradinate as 1 and orientation as 30 degrees can be specified by the "start" argument as: start:="[5,1,30]"
- Similarly the goal location can be specified using the "end" argument.
- The "RPM" argument can be used to specify both the RPM values.

### Github Link
https://github.com/pulkitmehta09/ENPM661-Project3/tree/main/proj3_phase2

![](https://github.com/pulkitmehta09/ENPM661-Project3/blob/main/proj3_phase2/Simulation%20Videos/output_sim.gif)
