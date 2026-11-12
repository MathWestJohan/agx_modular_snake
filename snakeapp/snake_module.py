import itertools
import math
import os

import agx
import agxSDK
import agxCollide
import agxRender

import snakeapp
from snakeapp import add, load_shape, create_constraint, create_visual
from snakeapp.snake_control import MAX_TORQUE

ASSETS = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'environment', 'assets')
print_density = 1240 * 0.75
servo_density = 1380

# The robot Snake() builds when no design is given:
#   'v5'  current design (assets/ssv5): a head and a tail, both standing on teeth, and a plate with
#         two teeth between each pair of modules. The snake stands on the tips of the teeth, 5 mm
#         below the modules. The head is bolted to an arm, so the arm end of the chain leads.
#   'v3'  the earlier bare modules (assets/ssv3), cradle end first
DESIGN = 'v5'

# V5 head and tail masses in kg, teeth included (e.g. weighed with the electronics or battery in
# them). None: printed plastic only, at print_density (head about 0.108 kg, tail about 0.085 kg).
# They sit far out on long levers, so they set how hard the servos must work.
HEAD_MASS = None
TAIL_MASS = None


def _shape(folder, name):
    return load_shape(os.path.join(ASSETS, folder, name))


# Meshes in metres, made by snakeapp/convert_snake_to_mesh.py. Module frame: hinge axis along z
# through the origin, the cradle's end plate toward -x, the arm's base plate toward +x.
cradle_shape = _shape('ssv3', 'cradle.obj')
bracket_shape = _shape('ssv3', 'bracket.obj')
servo_body_shape = _shape('ssv3', 'servo_body.obj')   # the same Parallax servo in both designs
servo_horn_shape = _shape('ssv3', 'servo_horn.obj')
module_length = 0.0786                                # V3: hinge to hinge

V5 = {name: _shape('ssv5', name + '.obj') for name in (
    'cradle_short', 'cradle_long', 'arm', 'arm_head', 'head', 'head_teeth', 'tail', 'tail_teeth', 'plate')}
ARM_FACE = 0.0375                         # hinge -> outer face of the arm's base plate
CRADLE_FACE = {'S': 0.0401, 'L': 0.0481}  # hinge -> outer face of the end plate, short / long cradle
PLATE_THICKNESS = 0.002                   # the plate between two modules (krybbe + plate + arm)
HEAD_LENGTH = 0.1093                      # the head's mounting face -> tip of its nose
TOOTH_DEPTH = 0.030                       # module axis -> tips of the teeth (the modules are 50 mm high)

HEAD_COLOR = agxRender.Color(0.3, 0.7, 0.35, 1.0)
TEETH_COLOR = agxRender.Color(0.75, 0.1, 0.1, 1.0)
PLATE_COLOR = agxRender.Color(0.95, 0.95, 0.95, 1.0)

_snake_ids = itertools.count()


class Snake(agxSDK.Assembly):

    def __init__(self, config='PYPYP', wheels=False, design=None, cradles=None):
        """config   joint types from the head back, e.g. 'PPYPP'.
        design   'v5' or 'v3' (default DESIGN).
        cradles  V5 only: 'S' (short) or 'L' (long) cradle for each module from the head back,
                 e.g. 'SSSSS' (the default) or 'LSSSS'. A long cradle makes its module 8 mm longer.
        wheels   True models passive wheels (or scales) under the snake: every rigid body gets its own
                 material, so the scene can give it low friction along the snake and normal friction
                 sideways (see course_scene.set_friction). Serpentine crawling (YYPYY) needs this.

        modules[0] is at the front (the head end), at the origin; the rest of the snake lies toward +x.
        nose is how far the front reaches ahead of the origin, bottom how far the snake reaches below it."""
        super().__init__()
        if len(config) < 5 or set(config) - set('PY'):
            raise ValueError("config must be at least 5 letters of P/Y, e.g. 'PPYPP', 'PYPYP', 'YYPYY'")
        self.config = config
        self.wheels = wheels
        self.design = design or DESIGN

        self.material = agx.Material('snake_material')
        self.material.getBulkMaterial().setDensity(print_density)
        self.servo_material = agx.Material('servo_material')
        self.servo_material.getBulkMaterial().setDensity(servo_density)

        self.modules = []
        self.extra_parts = []     # (body, geometry) of the head, tail, plates and teeth
        self.end_materials = []   # own materials of the head / tail when HEAD_MASS / TAIL_MASS are set
        if self.design == 'v3':
            self._build_v3()
        elif self.design == 'v5':
            self._build_v5(cradles or 'S' * len(config))
        else:
            raise ValueError("design must be 'v3' or 'v5'")

    def _materials(self, i):
        """Materials for module i's cradle body and arm body: shared, or one per body with wheels."""
        if not self.wheels:
            return self.material, self.material
        materials = []
        for part in ('cradle', 'bracket'):
            material = agx.Material('snake_{}_{}'.format(part, i))
            material.getBulkMaterial().setDensity(print_density)
            materials.append(material)
        return materials

    # -- V3: bare modules, cradle end first ---------------------------------------------------------
    def _build_v3(self):
        self.nose = 0.0411     # hinge -> end plate of module 0's cradle
        self.bottom = 0.0256
        self.yaw_sign = 1
        for i, kind in enumerate(self.config):
            cradle_material, bracket_material = self._materials(i)
            module = SnakeModule(cradle_material, self.servo_material, bracket_material)
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

    # -- V5: head, modules with plates in between, tail ---------------------------------------------
    def _build_v5(self, cradles):
        n = len(self.config)
        if len(cradles) != n or set(cradles) - set('SL'):
            raise ValueError("cradles must have one 'S' or 'L' per module, e.g. 'SSSSS'")
        self.nose = ARM_FACE + HEAD_LENGTH
        self.bottom = TOOTH_DEPTH
        # The head leads with an arm, where V3 leads with a cradle. The same yaw angle therefore
        # turns the snake the other way; the controller multiplies its steering by this. (The pitch
        # joints keep their sign: a negative angle still bends the body convex.)
        self.yaw_sign = -1

        # Build with the head toward +x (module frames as in V3), then turn the whole chain around so
        # that module 0 is at the origin and the rest toward +x, like V3.
        chain = agxSDK.Assembly()
        chain.setRotation(agx.EulerAngles(0, 0, math.pi))
        self.add(chain)

        x = 0.0
        for i, kind in enumerate(self.config):
            if i > 0:   # this module's arm + plate + previous module's cradle end plate
                x -= ARM_FACE + PLATE_THICKNESS + CRADLE_FACE[cradles[i - 1]]
            cradle_material, arm_material = self._materials(i)
            module = SnakeModule(cradle_material, self.servo_material, arm_material,
                                 cradle_shape=V5['cradle_long' if cradles[i] == 'L' else 'cradle_short'],
                                 bracket_shape=V5['arm_head' if i == 0 else 'arm'])
            module.setPosition(x, 0, 0)
            roll = math.pi / 2 if kind == 'P' else 0.0   # P: turned 90 deg about the axis, bends up/down
            module.setRotation(agx.EulerAngles(roll, 0, 0))
            chain.add(module)

            if i == 0:
                self._attach(module.bracket, 'head', self._end_material('head', HEAD_MASS, arm_material),
                             ARM_FACE, roll, HEAD_COLOR)
                self._attach(module.bracket, 'head_teeth', arm_material, ARM_FACE, roll, TEETH_COLOR)
            else:   # bolt the arm, with the plate, to the previous module's cradle
                self._attach(module.bracket, 'plate', arm_material, ARM_FACE, roll, PLATE_COLOR)
                merged = agx.MergedBody()
                merged.add(agx.MergedBodyEmptyEdgeInteraction(module.bracket, self.modules[-1].cradle))
                add(merged)
            if i == n - 1:
                tail_face = -CRADLE_FACE[cradles[i]]
                self._attach(module.cradle, 'tail', self._end_material('tail', TAIL_MASS, cradle_material),
                             tail_face, roll, HEAD_COLOR)
                self._attach(module.cradle, 'tail_teeth', cradle_material, tail_face, roll, TEETH_COLOR)
            self.modules.append(module)

        # The parts of one snake never touch each other: they are bolted together or turn on a
        # hinge, and five modules cannot bend far enough to reach a part further along.
        group = 'snake_{}'.format(next(_snake_ids))
        for body in self.bodies():
            for geometry in body.getGeometries():
                geometry.addGroup(group)
        snakeapp.sim().getSpace().setEnablePair(group, group, False)

    def _end_material(self, part, mass, printed):
        """Material for the head or tail: the printed one, or one whose density gives `mass` (kg,
        with the teeth). Same friction either way: the scenes set friction per material."""
        if mass is None:
            return printed
        teeth = V5[part + '_teeth'].getVolume() * print_density
        material = agx.Material('snake_{}_{}'.format(part, next(_snake_ids)))
        material.getBulkMaterial().setDensity((mass - teeth) / V5[part].getVolume())
        self.end_materials.append(material)
        return material

    def _attach(self, body, part, material, x, roll, color):
        """Add a V5 part to a module's body: its mounting face at x along the module axis, and upright
        (teeth down) whichever way the module is turned."""
        geometry = agxCollide.Geometry(V5[part].deepCopy())
        geometry.setMaterial(material)
        body.add(geometry, agx.AffineMatrix4x4.rotate(-roll, agx.Vec3.X_AXIS()) *
                 agx.AffineMatrix4x4.translate(x, 0, 0))
        create_visual(geometry, color)
        self.extra_parts.append((body, geometry))

    def bodies(self):
        for module in self.modules:
            yield module.cradle
            yield module.bracket

    def mass(self):
        return sum(body.getMassProperties().getMass() for body in self.bodies())

    def contact_parts(self):
        """(rigid body, geometry) for every part that touches the ground."""
        parts = []
        for module in self.modules:
            parts.append((module.cradle, module.cradle_geometry))
            parts.append((module.bracket, module.bracket_geometry))
        return parts + self.extra_parts


class SnakeModule(agxSDK.Assembly):

    def __init__(self, material, servo_material, bracket_material=None, cradle_shape=cradle_shape,
                 bracket_shape=bracket_shape):
        super().__init__()

        self.cradle_geometry = agxCollide.Geometry(cradle_shape.deepCopy())
        self.cradle_geometry.setMaterial(material)
        self.bracket_geometry = agxCollide.Geometry(bracket_shape.deepCopy())
        self.bracket_geometry.setMaterial(material if bracket_material is None else bracket_material)

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
