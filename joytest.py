import pygame
import time
pygame.init()
pygame.joystick.init()
joystick_count = pygame.joystick.get_count()
print(joystick_count)

joystick = pygame.joystick.Joystick(0)
joystick.init()
print(f"Name: {joystick.get_name()}")
print(f"Axis: {joystick.get_numaxes()}")

while True:
    for event in [pygame.event.wait(), ]:
        pass
    print(joystick.get_axis(3))
    time.sleep(0.5)