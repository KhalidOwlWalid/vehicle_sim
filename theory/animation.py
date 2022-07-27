import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation


xc = 0
yc = 0
steering_angle = 0 # delta
# slip_angle = 0 # beta
phi = 0 # yaw
max_steering_angle = 30 * np.pi/180

# Vehicle properties
L = 2
lr = 1.2
steer_flag = False

# Simulation properties
time = 0
sample_time = 0.01
t_max = 20.0

# Matplotlib
solution_xc = []
solution_yc = []
fig, ax = plt.subplots()
graph, = ax.plot(solution_xc, solution_yc)

class Fargs:

    def __init__(self, v, steering_angle):
        
        self.v = v
        self.steering_angle = steering_angle

def init():
    graph.set_data([], []) 
    return graph,

def animate(i, fargs):
    
    v = fargs.v
    steering_angle = fargs.steering_angle

    if steering_angle == 0:
        steering_angle = 0

    # Convert steering angle to radian
    steering_angle = steering_angle * np.pi/180

    x_dot = v * np.cos(phi)
    y_dot = v * np.sin(phi)
    # delta_dot = angular_velocity

    phi_dot = (v / L) * np.tan(steering_angle)
    
    # Update the current state of the car by integrating from velocity to position
    xc += x_dot * sample_time
    yc += y_dot * sample_time
    phi += phi_dot * sample_time

    if steering_angle < max_steering_angle:
        steering_angle += steering_angle * sample_time
    else:
        steering_angle = max_steering_angle

    solution_xc.append(xc)
    solution_xc.append(yc)

    graph.set_data(solution_xc, solution_yc)

    return graph,

fargs = [
    Fargs(
        v = np.pi,
        steering_angle = 0
    )]
        
anim = FuncAnimation(fig, animate, init_func=init,
                               frames=200, interval=20, fargs=fargs, blit=True)

plt.show()