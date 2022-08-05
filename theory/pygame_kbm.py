import pygame
from pygame.locals import (K_w, K_a, K_s, K_d, KEYDOWN, K_ESCAPE)
from pygame import transform

# TODO(Khalid): Create a color dictionary
# TODO(Khalid): Modularize this class cause it sucks

class Color:

    @staticmethod
    def white():
        return (255,255,255)

    @staticmethod
    def black():
        return (0,0,0)

    def red():
        return (255,0,0)

class Car(pygame.sprite.Sprite):
    def __init__(self, x: float, y: float, orientation: float, steering_angle: float, screen: pygame.display):
        super(Car, self).__init__()
        # Car properties
        self.CHASSIS_LENGTH = 80
        self.CHASSIS_WIDTH = 60
        self.WHEEL_WIDTH = 10
        self.WHEEL_DIAMETER = 15

        # Due to how pygame blits the top left corner on top of the surface top left corner
        # We calculated it this way to centralized it
        self.cog = ((self.CHASSIS_WIDTH - self.WHEEL_WIDTH)/2, (self.CHASSIS_LENGTH - self.WHEEL_DIAMETER)/2)

        # Due to how this is calculated, we shift xc and yc position to the center of chassis
        self.xc = x - self.CHASSIS_WIDTH/2
        self.yc = y - self.CHASSIS_LENGTH/2
        self.orientation = orientation
        self.steering_angle = steering_angle
        self.screen = screen
        

        self.surf = pygame.Surface(((self.CHASSIS_WIDTH), self.CHASSIS_LENGTH))
        self.surf.fill(Color.red())
        self.rect = self.surf.get_rect()

    def draw_rear_wheel(self, reflect: int=1) -> tuple[int]:
        """
        reflect (1,-1) where 1 is for right side, and -1 for left side
        """
        pygame.draw.rect(self.surf, color=Color.black(), rect=pygame.Rect(self.cog[0] + reflect * self.CHASSIS_WIDTH/4, self.cog[1] + self.CHASSIS_LENGTH/4, self.WHEEL_WIDTH, self.WHEEL_DIAMETER))
    
    def draw_front_wheel(self, reflect: int=1) -> tuple[int]:
        """
        reflect (1,-1) where 1 is for right side, and -1 for left side
        """
        pygame.draw.rect(self.surf, color=Color.black(), rect=pygame.Rect(self.cog[0] + reflect * self.CHASSIS_WIDTH/4, self.cog[1] - self.CHASSIS_LENGTH/4, self.WHEEL_WIDTH, self.WHEEL_DIAMETER))
    
    # def draw_front_wheel(self):

    #     p1 = (self.cog[0]  - self.WHEEL_WIDTH/2 - self.CHASSIS_WIDTH/4, self.cog[1]  + self.WHEEL_DIAMETER/2 - self.CHASSIS_LENGTH/4)
    #     p2 = (self.cog[0]  + self.WHEEL_WIDTH/2, self.cog[1]  + self.WHEEL_DIAMETER/2)
    #     p3 = (self.cog[0]  + self.WHEEL_WIDTH/2, self.cog[1]  - self.WHEEL_DIAMETER/2)
    #     p4 = (self.cog[0]  - self.WHEEL_WIDTH/2, self.cog[1]  - self.WHEEL_DIAMETER/2)

    #     pygame.draw.polygon(self.surf, color=Color.black(), points=(p1,p2,p3,p4))

pygame.init()

# Set up the drawing window
SCREEN_WIDTH = 500
SCREEN_HEIGHT = 500
screen = pygame.display.set_mode([SCREEN_WIDTH, SCREEN_HEIGHT])

# Run until the user asks to quit
running = True

SCREEN_CENTER = (SCREEN_WIDTH/2, SCREEN_HEIGHT/2)
car = Car(x=SCREEN_CENTER[0], y=SCREEN_CENTER[1], orientation=0, steering_angle=0, screen=screen)

entities = pygame.sprite.Group()
entities.add(car)

while running:

    # Did the user click the window close button?
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False

        if event.type == KEYDOWN:
            
            if event.key == K_ESCAPE:
                running = False

    screen.fill(Color.white())
    
    for entity in entities:
        entity.draw_rear_wheel(reflect=1)
        entity.draw_rear_wheel(reflect=-1)
        entity.draw_front_wheel(reflect=1)
        entity.draw_front_wheel(reflect=-1)
        screen.blit(entity.surf, (entity.xc, entity.yc))
    # Fill the background with white

    # Flip the display
    pygame.display.flip()

# Done! Time to quit.
pygame.quit()