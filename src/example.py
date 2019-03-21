import numpy as np
import RobotModels

l1 = 0
l2 = 0.2
l3 = 0.2

Link1 = RobotModels.Link(0,l1,np.pi/2)
Link2 = RobotModels.Link(0,l2,0)
Link3 = RobotModels.Link(0,l3,0)
bot = RobotModels.SerialLink([Link1, Link2, Link3], origin = [1,0,0])

Y = np.column_stack(np.deg2rad([45,30,50]))

X,R = bot.fkin(Y)
print("End effector position")
print(X)
print("End effector orientation")
print(R)
arm = bot.fkin_all(Y)
print("All joint forward kinemtatics")
print(arm)
ax = bot.plot3D(Y, elev = 45, azim = 240)
   