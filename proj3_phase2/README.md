# ENPM 661 Project-3 Phase2

## Team Members
### Pulkit Mehta (UID: 117551693)
### Anirudh Krishnan Komaralingam (UID: 117446432)


## INSTRUCTIONS TO RUN

Download/Clone the package into the workspace and build using catkin build or catkin_make.
Source the workspace and run:

```bash
roslaunch proj3_phase2 proj3phase2.launch start:="[5,1,30]" end:="[3,7,0]" RPM:="[10,10]" clearance:="0.1"
```

- A start location with x-coordinate as 5, y-cooradinate as 1 and orientation as 30 degrees can be specified by the "start" argument as: start:="[5,1,30]"
- Similarly the goal location can be specified using the "end" argument.
- The "RPM" argument can be used to specify both the RPM values.

