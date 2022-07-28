import pygame
from pygame.locals import (K_w, K_a, K_s, K_d, KEYDOWN, K_ESCAPE)

# TODO(Khalid): Create a color dictionary
# TODO(Khalid): Modularize this class cause it sucks
class Car(pygame.sprite.Sprite):
    def __init__(self):
        super(Car, self).__init__()

        self.x = 0
        self.y = 0

        # Create the body of the car
        CHASSIS_WIDTH = 60
        CHASSIS_HEIGHT = 80
        self.surf = pygame.Surface((CHASSIS_WIDTH, CHASSIS_HEIGHT))
        self.surf.fill((255, 255, 255))
        self.rect = self.surf.get_rect()

        # Create wheels
        WHEEL_WIDTH = 10
        WHEEL_HEIGHT = 15
        CHASSIS_CENTER = ((CHASSIS_WIDTH - WHEEL_WIDTH)/2, (CHASSIS_HEIGHT - WHEEL_WIDTH)/2)

        self.fl_surf = pygame.Surface((WHEEL_WIDTH, WHEEL_HEIGHT))
        self.fl_surf.fill((0,0,0))
        self.fl_rect = self.fl_surf.get_rect()
        self.surf.blit(self.fl_surf, dest=(CHASSIS_CENTER[0] - 15, CHASSIS_CENTER[1] - 20))

        self.fr_surf = pygame.Surface((WHEEL_WIDTH, WHEEL_HEIGHT))
        self.fr_surf.fill((0,0,0))
        self.fr_rect = self.fr_surf.get_rect()
        self.surf.blit(self.fr_surf, dest=(CHASSIS_CENTER[0] + 15, CHASSIS_CENTER[1] - 20))

        # Rear wheels
        pygame.draw.rect(self.surf, color=(0,0,0), rect=pygame.Rect(CHASSIS_CENTER[0] - 15, CHASSIS_CENTER[1] + 20, WHEEL_WIDTH, WHEEL_HEIGHT))
        pygame.draw.rect(self.surf, color=(0,0,0), rect=pygame.Rect(CHASSIS_CENTER[0] + 15, CHASSIS_CENTER[1] + 20, WHEEL_WIDTH, WHEEL_HEIGHT))

    # Move the sprite based on user keypresses
    def update(self, pressed_keys):
        if pressed_keys[K_w]:
            self.rect.move_ip(0, -1)
        if pressed_keys[K_s]:
            self.rect.move_ip(0, 1)
        if pressed_keys[K_a]:
            self.rect.move_ip(-1, 0)
        if pressed_keys[K_d]:
            self.rect.move_ip(1, 0)

            # Keep player on the screen
        if self.rect.left < 0:
            self.rect.left = 0
        if self.rect.right > SCREEN_WIDTH:
            self.rect.right = SCREEN_WIDTH
        if self.rect.top <= 0:
            self.rect.top = 0
        if self.rect.bottom >= SCREEN_HEIGHT:
            self.rect.bottom = SCREEN_HEIGHT

pygame.init()

# Set up the drawing window
SCREEN_WIDTH = 500
SCREEN_HEIGHT = 500
screen = pygame.display.set_mode([SCREEN_WIDTH, SCREEN_HEIGHT])

# Run until the user asks to quit
running = True

car = Car()

all_sprites = pygame.sprite.Group()
all_sprites.add(car)

while running:

    # Did the user click the window close button?
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False

        if event.type == KEYDOWN:
            
            if event.key == K_ESCAPE:
                running = False

    pressed_keys = pygame.key.get_pressed()
    car.update(pressed_keys)

    # Fill the background with white
    screen.fill((0, 0, 0))
    
    for entity in all_sprites:
        screen.blit(entity.surf, entity.rect)

    # Flip the display
    pygame.display.flip()

# Done! Time to quit.
pygame.quit()