# robolinks

## Create a link
Links are defined by 3 parameters from the DH-parameter convention (d,a,alpha).
To create a link:
```
Link1 = Link(d,a,alpha)
```

## Create Serial Link manipulator
To create a serial link manipulator, simple add links to a list and pass to SerialLink class:

```
Link1 = RobotModels.Link(0,0,np.pi/2)
Link2 = RobotModels.Link(0,0.2,0)
Link3 = RobotModels.Link(0,0.2,0)

bot = RobotModels.SerialLink([Link1, Link2, Link3], origin = [1,0,0])
```
### Optional Arguments
- origin: set origin of robot base link (defaults to [0,0,0])
- initY: initialize joints (defaults to 0 degrees for each joint)
- name: name of robot (default is "robot")

## Forward Kinematics
At this time, this class only supports forward kinematics of the end effector and all joint locations of the robot. If no angle is provided, it will use the current angle set internally after initialization or setting joints via ``` bot.setY(Q)```. <br />
<br />
To get the end effector position and orientation:
```
Q = np.column_stack(np.deg2rad([45,30,50]))
Position, Orientation = bot.fkin(Q)
```
To get array of all joint positions (and optionally orientations):
```
Positions = bot.fkin_all(Q) # does not return orientations by default
```
## Plot
The class supports basic plotting of the serial link manipulator:
```
ax = bot.plot3D(Q, elev = 45, azim = 240)
```
![robolink_plot](https://user-images.githubusercontent.com/22353511/54789327-5cefd800-4bef-11e9-9028-be62963c9095.png)
