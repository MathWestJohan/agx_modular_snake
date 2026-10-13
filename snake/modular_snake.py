import agx
import agxSDK
import agxOSG
import agxCollide
import agxRender
import math

class Snake(agxSDK.Assembly):
  def __init__(self, num_modules: int, pitch_only=False, with_camera=False):
    super().__init__()

class SnakeModule(agxSDK.Assembly):
  
  def __init__(self, material):
    super().__init__()
    
    # Load the mesh files for the cradle and bracket.
    cradle_mesh = agxCollide.Trimesh("environment/assets/SS_V3_krybbe.obj")
    bracket_mesh = agxCollide.Trimesh("environment/assets/SS_V3_brakett.obj")
    
    # For testing purposes, create simple box geometries to represent the cradle and bracket.
    cradle_geo = agxCollide.Geometry(agxCollide.Box(0.05, 0.05, 0.05)) # Rough initial estimates. Needs to be refined based on actual design.
    bracket_geo = agxCollide.Geometry(agxCollide.Box(0.05, 0.05, 0.01)) # Rough initial estimates. Needs to be refined based on actual design.
    
    # Rigid bodies
    self.cradle = agx.RigidBody(cradle_geo)
    self.bracket = agx.RigidBody(bracket_geo)
    
    # Set material
    self.cradle.getGeometries()[0].setMaterial(material)
    self.bracket.getGeometries()[0].setMaterial(material)
    
    # Visuals
    agxOSG.createVisual(self.cradle, agxRender.Color.Gray())
    agxOSG.createVisual(self.bracket, agxRender.Color.Gray())
    
    # Servo joint (hinge). Blue circle is servo output shaft.
    self.hinge = agx.Hinge(self.cradle, self.bracket)
    
    # Anchor point of the hinge. Check with CAD design
    self.hinge.setPosition(agx.Vec3(0, 0.05, 0))
    
    # Axis of rotation for the hinge. Check with CAD design.
    self.hinge.setAxis(agx.Vec3(0, 1, 0))
    
    # Enable the motor to simulate the servo's actuation.
    self.hinge.getMotor1D().setEnabled(True)
    
    # Set the ROM for the hinge. Adjust the range as needed based on the servo's specifications.
    # Currently set to -90 to 90 degrees.
    self.hinge.getRange1D().setRange(-math.pi/2, math.pi/2)
    
    # Full assembly.
    self.add(self.cradle)
    self.add(self.bracket)
    self.add(self.hinge)