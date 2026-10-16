import math
import os

import snakeapp
from snakeapp import load_shape
from snakeapp import create_constraint

import agx
import agxSDK
import agxCollide
import agxRender

from snakeapp import add, load_shape, create_constraint, create_visual
from snakeapp.snake_control import MAX_TORQUE

assets = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'environment', 'assets', 'ssv3')
module_length = 0.0786
print_density = 1240 * 0.75
servo_density = 1380

# Load the shapes for the snake components
cradle_shape = load_shape(os.path.join(assets, 'cradle.obj'))
bracket_shape = load_shape(os.path.join(assets, 'bracket.obj'))
servo_body_shape = load_shape(os.path.join(assets, 'servo_body.obj'))
servo_horn_shape = load_shape(os.path.join(assets, 'servo_horn.obj'))


class Snake(agxSDK.Assembly):

    def __init__(self, config='PYPYP'):
        super().__init__()
        if len(config) < 5 or set(config) - set('PY'):
            raise ValueError("config must be at least 5 letters of P/Y, e.g. 'PPYPP', 'PYPYP', 'YYPYY'")
        self.config = config

        self.material = agx.Material('snake_material')
        self.material.getBulkMaterial().setDensity(print_density)
        self.servo_material = agx.Material('servo_material')
        self.servo_material.getBulkMaterial().setDensity(servo_density)

        self.modules = []
        for i, kind in enumerate(config):
            module = SnakeModule(self.material, self.servo_material)
            module.setPosition(module_length * i, 0, 0)
            if kind == 'P':  # turn the module 90 deg about the snake's axis -> joint bends up/down
                module.setRotation(agx.EulerAngles(math.pi / 2, 0, 0))

            if i > 0:  # bolt this cradle to the previous module's bracket
                previous = self.modules[-1]
                merged = agx.MergedBody()
                merged.add(agx.MergedBodyEmptyEdgeInteraction(previous.bracket, module.cradle))
                add(merged)
                previous.bracket_geometry.setEnableCollisions(module.cradle_geometry, False)

            self.modules.append(module)
            self.add(module)
      

class SnakeModule(agxSDK.Assembly):
  
  def __init__(self, material, servo_material):
    super().__init__()
    
    self.cradle_geometry = agxCollide.Geometry(cradle_shape.deepCopy())
    self.cradle_geometry.setMaterial(material)
    self.bracket_geometry = agxCollide.Geometry(bracket_shape.deepCopy())
    self.bracket_geometry.setMaterial(material)
    
    servo = agxCollide.Geometry(servo_body_shape.deepCopy())
    servo.setMaterial(servo_material)
    servo.setEnableCollisions(False)
    horn = agxCollide.Geometry(servo_horn_shape.deepCopy())
    horn.setMaterial(servo_material)
    horn.setEnableCollisions(False)
    
    create_visual(self.cradle_geometry, agxRender.Color.Orange())
    create_visual(self.bracket_geometry, agxRender.Color.SkyBlue())
    create_visual(servo, agxRender.Color.Black())
    create_visual(horn, agxRender.Color.Gray())
    
    # two rigid bodies: cradle + servo body, and bracket + horn
    self.cradle = agx.RigidBody(self.cradle_geometry)
    self.cradle.add(servo)
    self.bracket = agx.RigidBody(self.bracket_geometry)
    self.bracket.add(horn)
    
    self.cradle_geometry.setEnableCollisions(self.bracket_geometry, False)

    # the servo: a motorised hinge along the shaft
    self.hinge = create_constraint(pos=agx.Vec3(0, 0, 0), axis=agx.Vec3(0, 0, -1),
                                   rb1=self.cradle, rb2=self.bracket, c=agx.Hinge)
    self.hinge.setCompliance(1E-12)
    self.hinge.getMotor1D().setCompliance(1E-10)
    self.hinge.getMotor1D().setEnable(True)
    self.hinge.getMotor1D().setForceRange(-MAX_TORQUE, MAX_TORQUE)
    self.hinge.getLock1D().setEnable(False)
    self.hinge.getRange1D().setRange(-math.pi / 2, math.pi / 2)

    self.add(self.cradle)
    self.add(self.hinge)
    self.add(self.bracket)