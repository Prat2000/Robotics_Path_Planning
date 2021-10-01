import math
import matplotlib.pyplot as plt
import numpy as np
import torch
from scipy.stats import norm

pi = math.pi

plt.gcf().canvas.mpl_connect('key_release_event',
                lambda event: [exit(0) if event.key == 'escape' else None])

def plot_circle(x, y, size, color="-b"): 
        deg = list(range(0, 360, 5))
        deg.append(0)
        xl = [x + size * math.cos(np.deg2rad(d)) for d in deg]
        yl = [y + size * math.sin(np.deg2rad(d)) for d in deg]
        plt.plot(xl, yl, color)

# obstacleList = [(5, 5, 1), (3, 6, 2), (3, 8, 2), (3, 10, 2), (7, 5, 2),
#                     (9, 5, 2), (8, 10, 1)]
obstacleList = [(5, 5, 1), (3, 8, 2), (9, 5, 2), (8, 10, 1)]

for (ox, oy, size) in obstacleList:
    plot_circle(ox, oy, size)

q_start = torch.tensor([1.,1.,pi/2],requires_grad = True)
q_goal = torch.tensor([5,2])

qs = q_start.detach().numpy()
qg = q_goal.detach().numpy()
plt.plot(qs[0],qs[1],'bo')
plt.plot(qg[0],qg[1],'go')

def d_goal(q,q_goal):
    x = q[0] - q_goal[0]
    y = q[1] - q_goal[1]
    d = torch.hypot(x,y)
    return d

def attractive_pot(q,q_goal,close):
    x = q[0] - q_goal[0]
    y = q[1] - q_goal[1]
    dist = d_goal(q,q_goal)
    theta_g = torch.atan2(y,x)
    ang = (theta_g - q[2])

    if dist>close:
        u_attr = 0.5*dist**2 + 0.03*ang**2
    else:
        u_attr = 0.5*dist**2 + 0.03*(ang**2)*close

    return u_attr

def repulsive_pot(q,obstacleList,fov):
    for i in range(len(obstacleList)):
        x = q[0] - obstacleList[i][0]
        y = q[1] - obstacleList[i][1]
        dist = np.hypot(x,y)
        mean = torch.atan2(y,x)
        std = torch.asin(obstaclelist[i][2]/dist)/3
        u_rep += 

    return u_rep

def path_planning(q_start,q_goal,obstacleList,fov,vel,dt,lr,close,thresh):
    
    max_itrs = 60
    path = [q_start]
    dg = d_goal(q_start,q_goal)
    q = q_start
    itr=0
    print("Beginning search...")

    while dg>=thresh and itr<max_itrs:
        u_attr = attractive_pot(q,q_goal,close)
        u_rep = repulsive_pot(q,obstacleList,fov)

        u_tot = - u_attr 
        # - u_rep
        u_tot.backward()

        with torch.no_grad():
            theta_grad = q_start.grad[2]
            q[2] = q[2] + lr*theta_grad
            q[0] = q[0] + vel*dt*torch.cos(q[2])
            q[1] = q[1] + vel*dt*torch.sin(q[2])

            print("{}  ------>   {} ----->  {}  ------->  {}".format(u_attr,u_rep,theta_grad,q[2]))

            q_start.grad.zero_()
            path.append(q)

        dg = d_goal(q,q_goal)
        itr = itr+1

        qp = q.detach().numpy()
        plt.plot(qp[0], qp[1], ".r")
        plt.pause(0.01)
    
    if dg<thresh:
        print("Reached goal!")
    else:
        print("Couldn't reach goal...")

    return path

path = path_planning(q_start,q_goal,obstacleList,120,2.,0.1,0.1,2.5,0.1)
plt.show()