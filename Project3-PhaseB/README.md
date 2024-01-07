# ENPM 661 Project-3 Phase2

## Team Members
### Nishant Pandey (UID: 119247556),npandey2
### Rishikesh Jadhav (UID: 119256534),rjadhav1

# Folder structure
proj3p2_nishant_rishikesh -> part2 ->src,world,launch
# Libraries used
numpy
matplotlib

## INSTRUCTIONS TO RUN

Download/Clone the package into the workspace and build using catkin build or catkin_make.
Source the workspace.

### 2-D Path Planning

To run A* path planning algorithm run the below from src folder the workspace:

```bash
$ cd proj3p2_nishant_rishikesh/part1/
$ python3 Phase2.py --Start="[0,0,30]" --End="[5,0,0]" --RPM="[10,15]"
```

- A start location with x-coordinate as 1, y-cooradinate as 1 and orientation as 30 degrees can be specified by the "--Start" argument as: --Start="[0,0,30]"
- Similarly the goal location can be specified using the "--End" argument.
- The "--RPM" argument can be used to specify both the RPM values.


### Gazebo Simulation

Make the node as executable by running the following from src folder in the workspace:
```bash
$ cd proj3_phase2/part2/scripts/
$ chmod +x node.py
```

To run the simulation:

```bash
roslaunch proj3_phase2 proj3phase2.launch start:="[0,0,30]" end:="[5,0,0]" RPM:="[8,8]" clearance:="0.0"
```

- A start location with x-coordinate as 5, y-cooradinate as 1 and orientation as 30 degrees can be specified by the "start" argument as: start:="[5,0,30]"
- Similarly the goal location can be specified using the "end" argument.
- The "RPM" argument can be used to specify both the RPM values.

### Github Link

https://github.com/nishantpandey4/prj3_phase2.git

### Video link 
https://drive.google.com/drive/folders/17qJBgX0-oyyZXjs0hwa5U_dFdOoDMzCe?usp=share_link 
