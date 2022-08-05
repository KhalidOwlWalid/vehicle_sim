import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

class Fargs:

    def __init__(self, v, steering_angle):
        
        self.v = v
        self.steering_angle = steering_angle

class KinematicBicycleModel:

    def __init__(self):
        
        self.xc = 0
        self.yc = 0
        self.steering_angle = 0 # delta
        # self.slip_angle = 0 # beta
        self.phi = 0 # yaw
        self.max_steering_angle = 30 * np.pi/180

        # Vehicle properties
        self.L = 2
        self.lr = 1.2
        self.steer_flag = False

        # Simulation properties
        self.time = 0
        self.sample_time = 0.01
        self.t_max = 20.0

        # Matplotlib
        self.solution_xc = []
        self.solution_yc = []
        self.fig, self.ax = plt.subplots()
        self.graph, = self.ax.plot(self.solution_xc, self.solution_yc)

    def init(self):
        self.graph.set_data([], []) 
        return self.graph,

    def animate(self,i,fargs) -> tuple[float, float]:
        
        v = fargs.v
        steering_angle = fargs.steering_angle

        if steering_angle == 0:
            self.steering_angle = 0

        # Convert steering angle to radian
        steering_angle = steering_angle * np.pi/180

        x_dot = v * np.cos(self.phi)
        y_dot = v * np.sin(self.phi)
        # delta_dot = angular_velocity
    
        phi_dot = (v / self.L) * np.tan(self.steering_angle)
        
        # Update the current state of the car by integrating from velocity to position
        self.xc += x_dot * self.sample_time
        self.yc += y_dot * self.sample_time
        self.phi += phi_dot * self.sample_time

        self.ax.set_xlim(self.xc - 20, self.xc + 20)
        self.ax.set_ylim(self.yc - 20, self.yc + 20)

        if self.steering_angle < self.max_steering_angle:
            self.steering_angle += steering_angle * self.sample_time
        else:
            self.steering_angle = self.max_steering_angle

        self.solution_xc.append(self.xc)
        self.solution_yc.append(self.yc)

        self.graph.set_data(self.solution_xc, self.solution_yc)

        print(self.xc, self.yc)
        return self.graph,

kmb = KinematicBicycleModel()

fargs = [Fargs(
    v=np.pi,
    steering_angle=20
)]  

anim = FuncAnimation(kmb.fig, kmb.animate, init_func=kmb.init,
                               frames=int(kmb.t_max/kmb.sample_time), fargs=fargs, interval=20, blit=True)

plt.show()