# Simple pygame program
import os

# Import and initialize the pygame library
import pygame
import numpy as np
from datetime import datetime

pygame.init()

class Color:
    
    @staticmethod
    def red():
        return (255,0,0)

    @staticmethod
    def white():
        return (255,255,255)

    @staticmethod
    def black():
        return (0,0,0)

class KinematicBicycleModel:

    def __init__(self, xc: float, yc: float, steering_angle: float, yaw: float, screen: pygame.display) -> None:
        
        self.xc = xc
        self.yc = yc
        self.steering_angle = steering_angle
        self.yaw = yaw
        self.distance_travelled = 0

        # Car properties
        self.L = 2.0
        self.lr = 1.2

        # Simulation properties
        self.time = 0
        self.time_max = 30.0
        self.sample_time = 0.01
        self.screen = screen

    def update(self, v : float, angular_velocity : float) -> tuple[float, float]:

        xc_dot = v * np.cos(self.yaw)
        yc_dot = v * np.sin(self.yaw)
        delta_dot = angular_velocity # w
        yaw_rate = (v / self.L) * np.tan(self.steering_angle)
        
        # Update the current state of the car by integrating from velocity to position
        self.xc += xc_dot * self.sample_time
        self.yc += yc_dot * self.sample_time
        self.yaw += yaw_rate * self.sample_time
        self.steering_angle =  delta_dot * self.sample_time

        self.time += self.sample_time

    def draw_circle(self):
        return pygame.draw.circle(self.screen, Color.red(), (self.xc, self.yc), 5)

    def calculate_distance(self, x_prev: float, x_cur: float, y_prev: float, y_cur: float) -> tuple[float, float, float, float]:
        return np.sqrt((x_cur - x_prev) ** 2 + (y_cur - y_prev) ** 2)


SCREEN_WIDTH = 500
SCREEN_HEIGHT = 500
# Setup the window we'll use for drawing
screen = pygame.display.set_mode([SCREEN_WIDTH, SCREEN_HEIGHT])

model = KinematicBicycleModel(SCREEN_WIDTH/2, SCREEN_HEIGHT/4, 0, 0, screen=screen)

# Run until the user asks us to quit
running = True

# TODO(Khalid): Data collection
tmp_data_list = []

while running:

    # Did the user click the window close button?
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False

    # Update FPS rate
    clock = pygame.time.Clock()
    clock.tick(1/model.sample_time)

    # Update model information
    font = pygame.font.SysFont(None, 24)
    xc_surf = font.render(f'xc : {np.round(model.xc, 3)}', True, Color.black())
    yc_surf = font.render(f'yc : {np.round(model.yc, 3)}', True, Color.black())
    steering_surf = font.render(f'Distance travelled : {np.round(model.distance_travelled, 3)}', True, Color.black())
    time_surf = font.render(f'Time : {np.round(model.time, 3)}', True, Color.black())

    surface_text_list = [xc_surf, yc_surf, steering_surf, time_surf]

    # Fill the background with white
    screen.fill(Color.white())

    for i, text in enumerate(surface_text_list):
        screen.blit(text, (20, i * 20))

    # Store previous xc and yc position
    x_prev, y_prev = model.xc, model.yc
    model.update(30, 0)
    model.distance_travelled += model.calculate_distance(x_prev, model.xc, y_prev, model.yc)

    tmp_data_list.append((np.round(model.time, 3), np.round(model.xc, 3), np.round(model.yc, 3), np.round(model.distance_travelled, 3)))

    model.draw_circle()

    # Flip the display
    pygame.display.flip()

# Extract data
tmp_data_list = np.array(tmp_data_list)

# File properties
now = datetime.now()
cwd = os.getcwd()
filename = now.strftime("%d-%m-%Y-%H_%M_%S")
file = os.path.join(cwd,"theory", "data", f"{filename}.csv")
header = "Time, xc, yc, Distance travelled"
np.savetxt(file, tmp_data_list, delimiter=',', fmt=('%5.4f', '%5.4f', '%5.4f', '%5.4f'), header=header, comments="")

# We're done, so we can quit now.
pygame.quit()
