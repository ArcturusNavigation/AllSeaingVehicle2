import pygame
import math

# Initialize pygame
pygame.init()

# Window settings
WIDTH, HEIGHT = 800, 600
win = pygame.display.set_mode((WIDTH, HEIGHT))
pygame.display.set_caption("Boat Simulator")

# Colors
WHITE = (255, 255, 255)
BLUE_WATER = (0, 119, 190)
BOAT_COLOR = (0, 100, 255)
RED_BUOY = (255, 0, 0)
GREEN_BUOY = (0, 255, 0)

# Boat settings
DAMPING = 0.98
ANGULAR_DAMPING = 0.96

class Boat:
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.angle = 0
        self.velocity = pygame.Vector2(0, 0)
        self.angular_velocity = 0
        self.left_thruster = 0
        self.right_thruster = 0

    def update(self):
        force = self.left_thruster + self.right_thruster
        rotational_force = self.right_thruster - self.left_thruster
        self.velocity.x += math.sin(math.radians(self.angle)) * force
        self.velocity.y -= math.cos(math.radians(self.angle)) * force
        self.angular_velocity += rotational_force
        self.velocity *= DAMPING
        self.angular_velocity *= ANGULAR_DAMPING
        self.x += self.velocity.x
        self.y += self.velocity.y
        self.angle += self.angular_velocity

    def draw(self, window):
        # Define a speedboat-like shape
        boat_shape = [
            (0, 75),
            (10, 50),
            (20, 25),
            (30, 15),
            (35, 10),
            (50, 10),
            (55, 15),
            (65, 25),
            (75, 50),
            (85, 75),
            (50, 90)
        ]

        # Transform points based on boat's position and angle
        rotated_shape = []
        for point in boat_shape:
            x = self.x + math.cos(math.radians(self.angle)) * (point[0] - 42.5) - math.sin(math.radians(self.angle)) * (point[1] - 50)
            y = self.y + math.sin(math.radians(self.angle)) * (point[0] - 42.5) + math.cos(math.radians(self.angle)) * (point[1] - 50)
            rotated_shape.append((x, y))
        
        pygame.draw.polygon(window, BOAT_COLOR, rotated_shape)

def draw_buoys(window):
    pygame.draw.circle(window, RED_BUOY, (250, 250), 10)
    pygame.draw.circle(window, GREEN_BUOY, (550, 350), 10)

def main():
    clock = pygame.time.Clock()
    boat = Boat(WIDTH / 2, HEIGHT / 2)

    run = True
    while run:
        win.fill(BLUE_WATER)

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                run = False

            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_a:
                    boat.left_thruster = 1
                if event.key == pygame.K_d:
                    boat.right_thruster = 1

            if event.type == pygame.KEYUP:
                if event.key == pygame.K_a:
                    boat.left_thruster = 0
                if event.key == pygame.K_d:
                    boat.right_thruster = 0

        boat.update()
        boat.draw(win)
        draw_buoys(win)

        pygame.display.flip()
        clock.tick(60)

    pygame.quit()

if __name__ == "__main__":
    main()