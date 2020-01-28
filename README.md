# Path and Trajectory Planning - Robot Mechanics and Control

### 1. Environment Used
- MATLAB R2018a
- Toolbox: [Robotics Toolbox](http://petercorke.com/wordpress/toolboxes/robotics-toolbox)

### 2. Robot Model

- Proposed Mechanism: 3-link cylindrical manipulator (RPP)
- Workspace is cylindrical 

### 2.1 Forward Kinematics:
- Axis Assignment
![RPP Forward Kinematics](https://i.imgur.com/zEUC3XY.png)
- DH parameters
![DH params - RPP](https://i.imgur.com/YlvQyP1.png)
- Transformations Matrices
![Transformation Matrices](https://i.imgur.com/P27ttqn.png)

- Defining Robot Model in MATLAB
```sh
%% Robot Model Definition
d1 = 5;
L1 = Revolute('a', 0, 'd', d1, 'qlim', [(1*pi/4),(3*pi/4)],'standard');
L2 = Prismatic('a', 0, 'alpha', -pi/2, 'qlim',[5,200],'standard');
L3 = Prismatic('a', 0, 'qlim',[2,abs(R)],'standard');

robot1 = SerialLink( [L1 L2 L3],'name', 'RPP/Cake Robot');
text(0,0,0,' RPP ROBOT')
robot.teach('rpy')
```

### 2.2 Inverse Kinematics
- Inverse Kinematics Equations
![Inverse Kinematics](https://i.imgur.com/pdEbayO.png)


### 3. Workspace
- Proposed Workspace: Cake-shaped Workspace
- Cake-shaped Workspace is drawn by generating every possible values of EE Pose (x, y, z of T(0,3))
- When plotting each point using point3, check det(Jacobian) for any singularity)
- Singularity for RPP occurs when 3rd link variable becomes zero
|Jacob| = d3
- Boundary: Drawn using meshgrid
-- Radius is 282 units
-- Width is 400 units
-- Height is 200 units
-- H/W ratio: 0.5
-- Theta = pi/2

![Robot Workspace](https://i.imgur.com/c9aK1yD.png)

### 4. Path and Trajectory Plotting in Restricted Workspace
###### Steps Involved:
- First Get start and goal EE position from user in terms of cartesian coordinates
- Using inverse kinematics to get joint configuration values (initial and start)
- Using ```mstraj``` (Multi-segment multi-axis trajectory) to generate values of - joint configuration for the trajectory
- Loop through each joint configuration and check for singularity/joint limits. Quit the program if required
- Draw the trajectory using ```plot3()``` function
- Animate the robot to follow the trajectory

![Robot Path and Trajectory](https://i.imgur.com/s118v7c.png)

### 4.1 Velocity and Acceleration Graphs
- Using ```jtaj``` command of robotics toolbox to get ```q, qd, qdd```

![Velocity and Acceleration Graphs](https://i.imgur.com/COqMyzI.png)

### 5. Overall Program Flow
- Get Start and Goal cartesian coordinates from the user
- Map those coordinates to EE pose/axis 
- Use inverse kinematics to get Start and Goal joint configuration
- Use mstraj to get various joint configuration between start and goal config
- If joint limit exceeds, quit the program
- Robot Animation: using robot.plot 
- Drawing Trajectory: Use fwd kinematics to get x,y,z 
- for each joint config
- Use plot3 to draw points
- Display the coordinates (start, goal)
- Calculating time to each goal using cputime
- Ask again for another goal position (Homing in start ONLY)

### 5.1 Sample End and Start Cartesian Coordinates in the Workspace
- [-200,50,120]
- [-255,70,140]
- [-150,-70,140]

