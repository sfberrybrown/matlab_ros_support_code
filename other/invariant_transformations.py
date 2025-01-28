import numpy as np
import math
from collections import namedtuple

from spatialmath import *
from spatialmath.base import *
from roboticstoolbox import *
import roboticstoolbox.tools.trajectory as tr

import matplotlib.pyplot as plt

### Ways to generate a series of SE3s
## 1.A. Direct Way with CTRAJ

# Create start/end configs, i.e. a translation about the x-axis
T0 = SE(3)
T1 = SE3.Rx(0,t=[0,0,-5])
T = ctraj(T0,T1,100) # Call ctraj: returns SE3 x n

# Animation 
# a. Take first and last frame and use animate
T0.animate(start=T1,frame='A',color='green',dims=[-10,10])

# b. Animate each frame
fig = plt.figure()
axes = plt.axes( xlim=(-5,5), ylim=(-5,5) )

nth = 10
dims = [-5,5]

fig = plt.figure()
for i in range(0,len(T),nth):
    T[i+1].animate(start=T[i],frame=str(i))
    #print(i)
    fig.clear()
    


## 1.B. Indirectly via joint angles
# Initial angle transformation
q0 = [0,0,0]
qf = [-pi/2,0,0]

out  = jtraj(q0,qf,100)    # Angle interporlation
outT = SE3(out.q)          # Convert back to SE3

### 2.A transform an entire sequence of points
# Create points moving from origin to (1,1)
p0 = np.array([0,0,0])
pf = np.array([1,1,0])

# Interpolate
out = tr.jtraj(p0, pf, 100)
fig = plt.figure(1)
plot(out.q)
plt.show()

# Transform the points
T=S03.Rz(90,"deg")

pnew = np.zeros(len(out.q),3,1) # i.e. 100x3x1
for i in range(len(out.q)):
    pnew[i,:,:]=T*out.q[i]

