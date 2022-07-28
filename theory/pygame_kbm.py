import pygame

# TODO(Khalid): Create a color dictionary
class Car(pygame.sprite.Sprite):
    def __init__(self):
        super(Car, self).__init__()

        # Create the body of the car
        CHASSIS_WIDTH = 60
        CHASSIS_HEIGHT = 80
        self.chassis_surf = pygame.Surface((CHASSIS_WIDTH, CHASSIS_HEIGHT))
        self.chassis_surf.fill((255, 255, 255))
        self.rect = self.chassis_surf.get_rect()

        # Create wheels
        WHEEL_WIDTH = 10
        WHEEL_HEIGHT = 15
        CHASSIS_CENTER = ((CHASSIS_WIDTH - WHEEL_WIDTH)/2, (CHASSIS_HEIGHT - WHEEL_WIDTH)/2)

        self.fl_surf = pygame.Surface((WHEEL_WIDTH, WHEEL_HEIGHT))
        self.fl_surf.fill((0,0,0))
        self.fl_rect = self.fl_surf.get_rect()
        self.chassis_surf.blit(self.fl_surf, dest=(CHASSIS_CENTER[0] - 15, CHASSIS_CENTER[1] - 20))

        self.fr_surf = pygame.Surface((WHEEL_WIDTH, WHEEL_HEIGHT))
        self.fr_surf.fill((0,0,0))
        self.fr_rect = self.fr_surf.get_rect()
        self.chassis_surf.blit(self.fr_surf, dest=(CHASSIS_CENTER[0] + 15, CHASSIS_CENTER[1] - 20))

        # Rear wheels
        pygame.draw.rect(self.chassis_surf, color=(0,0,0), rect=pygame.Rect(CHASSIS_CENTER[0] - 15, CHASSIS_CENTER[1] + 20, WHEEL_WIDTH, WHEEL_HEIGHT))
        pygame.draw.rect(self.chassis_surf, color=(0,0,0), rect=pygame.Rect(CHASSIS_CENTER[0] + 15, CHASSIS_CENTER[1] + 20, WHEEL_WIDTH, WHEEL_HEIGHT))
        
pygame.init()

# Set up the drawing window

SCREEN_WIDTH = 500
SCREEN_HEIGHT = 500
screen = pygame.display.set_mode([SCREEN_WIDTH, SCREEN_HEIGHT])

# Run until the user asks to quit
running = True

car = Car()

while running:

    # Did the user click the window close button?
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False

    # Fill the background with white
    screen.fill((0, 0, 0))

    screen.blit(car.chassis_surf, ((SCREEN_HEIGHT - car.chassis_surf.get_height())/2, (SCREEN_WIDTH - car.chassis_surf.get_width())/2))
    # Flip the display
    pygame.display.flip()

# Done! Time to quit.
pygame.quit()