import math
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))

import agx
import agxCollide
import agxRender

from snakeapp import add, add_event_listener, create_visual, init_camera
from snakeapp.snake_module import Snake
from snakeapp.snake_control import SnakeControl

CONFIG = 'PYPYP'   # 'PPYPP', 'PYPYP' or 'YYPYY'


def build_scene():

    ground = agxCollide.Geometry(agxCollide.Box(2, 2, 0.05), agx.AffineMatrix4x4.translate(0, 0, -0.05))
    create_visual(ground, agxRender.Color.Green())   # top surface at z = 0
    add(ground)

    robot = Snake(CONFIG)
    robot.setPosition(0, 0, 0.03)
    add(robot)

    controller = SnakeControl(robot)
    if CONFIG == 'PYPYP':
        controller.init_sidewinding(math.pi / 9, math.pi * 2 / 3, 16.0)
    else:
        controller.init_turning(math.pi / 9, math.pi * 2 / 3, 8.0, 0.0, 0.0)  # travelling wave on the pitch joints
    add_event_listener(controller)

    init_camera(eye=agx.Vec3(-0.6, -0.8, 0.5), center=agx.Vec3(0.15, 0, 0))