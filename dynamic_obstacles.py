import numpy as np
import matplotlib.pyplot as plt 
from math import sqrt
from matplotlib.animation import FuncAnimation as f_anim
from matplotlib import rc

N = 20
world = np.zeros([N,N])
start = np.array([0,0])
goal = np.array([N,N])
obstacles = np.array([[4,5],[8,10],[12,7],[6,10],[7,7],[12,15]])
velocity = np.array([[1,1],[-1,0],[1,2],[1,2],[1,1],[-1,-1]])


k_attr = 1.5
k_rep = 1.5
d_max = 3
step = 1 #duration of 1 step

no_iters = 30

def simulate(x,y):
    fig = plt.figure()
    plt.xlim([-1,N+1])
    plt.ylim([-1,N+1])
    plt.plot(start[0], start[1], 'b^', markersize = 8)
    plt.plot(goal[0], goal[1], 'g*', markersize = 12)
    graph, = plt.plot([], [], 'yo', markersize = 3)

    def animate(i):
        graph.set_data(x[:i+1], y[:i+1])
        graph.set_data(x_obs_list[:i+1], y_obs_list[:i+1])
        return graph

    anim = f_anim(fig, animate, frames=no_iters, interval=200)    
    plt.show()

def f_attr(r):
    #r -> current location (x_r, y_r)
    f = np.subtract(goal, r)
    f_mag = sqrt(f[0]**2 + f[1]**2)
    f_theta = np.arctan(f[1]/f[0])
    f_x = k_attr*np.cos(f_theta) 
    f_y = k_attr*np.sin(f_theta)

    return np.array([f_x,f_y])

def f_rep(r):
    #r -> current position (x_r, y_r)
    f_x = 0
    f_y = 0

    for obs in obstacles:
        diff = np.subtract(r, obs)
        diff_mag = sqrt(diff[0]**2 + diff[1]**2)
        if diff_mag < d_max:
            alpha = (1/diff_mag) - (1/d_max)
            s_i_theta = np.arctan(diff[1]/diff[0])
            s_i_x = -k_rep*alpha*np.cos(s_i_theta)
            s_i_y = -k_rep*alpha*np.sin(s_i_theta)

            f_x = f_x + s_i_x
            f_y = f_y + s_i_y
        
    return np.array([f_x,f_y])

def move(r):
    #r -> current position (x_r, y_r)
    f_total = f_attr(r) + f_rep(r)
    # m = 1
    # a = f_total/m
    # s = at (displacement) = f_total
    s = f_total*step 
    r_new = np.add(r, s) 

    return r_new            


if __name__ == "__main__":
    
    cur = start
    x = []
    y = []

    x_obs_list = []
    y_obs_list = []   

    for _ in range(no_iters):
        x_obs_list = np.array([ob[0] for ob in obstacles])
        y_obs_list = np.array([ob[1] for ob in obstacles])

        obstacles = obstacles + velocity
        x.append(cur[0])
        y.append(cur[1])
        cur = move(cur)
        plt.xlim([-1,21])
        plt.ylim([-1,21])

        plt.scatter([start[0],goal[0]], [start[1],goal[1]])
        plt.scatter(cur[0],cur[1],s = 25**2, marker=r'$\clubsuit$')
        plt.scatter(x_obs_list, y_obs_list)     
        plt.pause(0.5)
        plt.clf()
    print(cur)

    #simulate(x,y)
    plt.show()
    
    

    
