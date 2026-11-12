"""Convert the CAD exports (millimetres, CAD frames) to the meshes the simulation loads (metres).

    python snakeapp/convert_snake_to_mesh.py          # both designs
    python snakeapp/convert_snake_to_mesh.py v5       # only one of them

Module frame (both designs): hinge axis along z through the origin, the cradle's end plate toward -x,
the arm's base plate toward +x.
"""
import os
import sys

ASSETS = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'environment', 'assets')
FOLDER = os.path.join(ASSETS, 'ssv3')
FOLDER_V5 = os.path.join(ASSETS, 'ssv5')
MM = 0.001

# p_module = R * p_cad + t   (mm), measured from the CAD files:
# cradle : idler boss at (x=0, y=-11.6), joint mid-plane at z=21.1, end plate at y=25.5..29.5
CRADLE = (((0, -1, 0), (1, 0, 0), (0, 0, 1)), (-11.6, 0.0, -21.1))
# servo  : shaft at (x=0, y=30.1); sits on the cradle floor (z=4), shaft over the idler boss
SERVO = (((0, 1, 0), (-1, 0, 0), (0, 0, 1)), (-30.1, 0.0, -17.1))
# bracket: joint holes at (x=0, z=34.5); the flange with the horn screw holes (y<0) is on the horn side
BRACKET = (((0, 0, -1), (1, 0, 0), (0, -1, 0)), (34.5, 0.0, 0.0))


def read_obj(path):
    verts, faces = [], []
    with open(path, errors='ignore') as f:
        for line in f:
            if line.startswith('v '):
                verts.append(tuple(float(x) for x in line.split()[1:4]))
            elif line.startswith('f '):
                idx = [int(t.split('/')[0]) - 1 for t in line.split()[1:]]
                for k in range(1, len(idx) - 1):
                    faces.append((idx[0], idx[k], idx[k + 1]))
    return verts, faces


def transform(verts, tr):
    r, t = tr
    return [tuple(sum(r[i][k] * v[k] for k in range(3)) + t[i] for i in range(3)) for v in verts]


def write_obj(path, verts, faces, comment):
    used = sorted({i for f in faces for i in f})
    new_index = {old: new for new, old in enumerate(used)}
    with open(path, 'w') as out:
        out.write('# {} - metres, SS module frame\n'.format(comment))
        for i in used:
            out.write('v {:.7f} {:.7f} {:.7f}\n'.format(*(c * MM for c in verts[i])))
        for f in faces:
            out.write('f {} {} {}\n'.format(*(new_index[i] + 1 for i in f)))
    print('wrote', path, len(used), 'vertices', len(faces), 'triangles')


def split_servo(verts, faces):
    """The servo export has two closed shells: the body and the horn (all of it above z = 37 mm)."""
    key = [tuple(round(c, 4) for c in v) for v in verts]
    parent = {}

    def find(a):
        while parent.setdefault(a, a) != a:
            parent[a] = parent[parent[a]]
            a = parent[a]
        return a

    for f in faces:
        r0 = find(key[f[0]])
        for i in f[1:]:
            r = find(key[i])
            if r != r0:
                parent[r] = r0
    lowest = {}
    for f in faces:
        root = find(key[f[0]])
        lowest[root] = min(lowest.get(root, 1e9), min(verts[i][2] for i in f))
    body, horn = [], []
    for f in faces:
        (horn if lowest[find(key[f[0]])] > 37.0 else body).append(f)
    return body, horn


def cad(name_options, folder=FOLDER):
    for name in name_options:
        path = os.path.join(folder, name)
        if os.path.exists(path):
            return path
    raise FileNotFoundError('none of {} in {}'.format(name_options, folder))


# --- V5 -------------------------------------------------------------------------------------------
# The V5 cradles and arms are drawn in the same CAD frames as V3, so CRADLE and BRACKET apply:
#   SS_V5_krybbe_kort  = the V3 cradle with a 3 mm end plate (y = 25.5..28.5) instead of 4 mm
#   SS_V5_krybbe_lang  = the short cradle with the end plate moved 8 mm further out (y = 33.5..36.5)
#   SS_V5_arm_vanlig   = the V3 bracket, unchanged
#   SS_V5_arm_head     = the same arm with a cable notch in the base plate; it carries the head
# Head and tail: a 50 x 50 x 3 mm flange at x = 0..3 with the body toward -x, bottom at z = -25.
# Mellomplate: 2 mm plate (x = 0..2) between two modules, with a tooth under each bottom corner (z -30).
# Tenner: teeth that clip under a bottom corner of the head or tail box (corner at the origin, box at
# x > 0, y > 0, z > 0); the tip is 5 mm below the box. The _head_L / _head_R ones (notched wall) go on
# the head's front corners, the plain ones everywhere else. Every mounting plate has the same 34.5 mm
# square bolt pattern, centred on the module axis.
#
# Output frames (the module code places these):
#   head.obj, head_teeth.obj  mounting face at x = 0, head toward +x, upright (bottom z = -25 mm)
#   tail.obj, tail_teeth.obj  mounting face at x = 0, tail toward -x, upright
#   plate.obj                 face at x = 0 against the arm, plate toward +x, teeth down
HEAD_MOUNT = (((-1, 0, 0), (0, -1, 0), (0, 0, 1)), (3.0, 0.0, 0.0))   # turned 180 deg about z
TAIL_MOUNT = (((1, 0, 0), (0, 1, 0), (0, 0, 1)), (-3.0, 0.0, 0.0))
IDENTITY = (((1, 0, 0), (0, 1, 0), (0, 0, 1)), (0.0, 0.0, 0.0))
BOTTOM = -25.0
# rotation about z that turns a tooth (box toward +x, +y) to a corner whose box lies toward (sx, sy)
TOOTH_TURN = {(1, 1): ((1, 0, 0), (0, 1, 0), (0, 0, 1)), (-1, 1): ((0, -1, 0), (1, 0, 0), (0, 0, 1)),
              (-1, -1): ((-1, 0, 0), (0, -1, 0), (0, 0, 1)), (1, -1): ((0, 1, 0), (-1, 0, 0), (0, 0, 1))}
# (tooth file, corner x, corner y, side of the corner the box is on), in the head / tail CAD frame;
# the corners are where the M2 holes in the box walls are (5 mm from the corner, 5 mm above the bottom)
HEAD_TEETH = [('SS_V5_tenner.obj', -10.0, -25.0, (-1, 1)), ('SS_V5_tenner.obj', -10.0, 25.0, (-1, -1)),
              ('SS_V5_tenner_head_L.obj', -63.0, -25.0, (1, 1)), ('SS_V5_tenner_head_R.obj', -63.0, 25.0, (1, -1))]
TAIL_TEETH = [('SS_V5_tenner.obj', -10.0, -30.0, (-1, 1)), ('SS_V5_tenner.obj', -10.0, 30.0, (-1, -1)),
              ('SS_V5_tenner.obj', -59.0, -30.0, (1, 1)), ('SS_V5_tenner.obj', -59.0, 30.0, (1, -1))]


def teeth(corners):
    """All the teeth of the head or tail as one mesh, in that part's CAD frame."""
    verts, faces = [], []
    for name, cx, cy, side in corners:
        v, f = read_obj(cad([name], FOLDER_V5))
        v = transform(v, (TOOTH_TURN[side], (cx, cy, BOTTOM)))
        faces += [tuple(i + len(verts) for i in face) for face in f]
        verts += v
    return verts, faces


def convert_v5():
    out = lambda name: os.path.join(FOLDER_V5, name)
    for cad_name, name, tr in [('SS_V5_krybbe_kort.obj', 'cradle_short.obj', CRADLE),
                               ('SS_V5_krybbe_lang.obj', 'cradle_long.obj', CRADLE),
                               ('SS_V5_arm_vanlig.obj', 'arm.obj', BRACKET),
                               ('SS_V5_arm_head.obj', 'arm_head.obj', BRACKET),
                               ('SS_V5_head.obj', 'head.obj', HEAD_MOUNT),
                               ('SS_V5_tail.obj', 'tail.obj', TAIL_MOUNT),
                               ('SS_V5_mellomplate.obj', 'plate.obj', IDENTITY)]:
        v, f = read_obj(cad([cad_name], FOLDER_V5))
        write_obj(out(name), transform(v, tr), f, '{} ({})'.format(cad_name[:-4], name[:-4]))
    v, f = teeth(HEAD_TEETH)
    write_obj(out('head_teeth.obj'), transform(v, HEAD_MOUNT), f, 'SS_V5_tenner x2, _head_L, _head_R (head teeth)')
    v, f = teeth(TAIL_TEETH)
    write_obj(out('tail_teeth.obj'), transform(v, TAIL_MOUNT), f, 'SS_V5_tenner x4 (tail teeth)')


def convert_v3():
    v, f = read_obj(cad(['SS_V3_krybbe.obj']))
    write_obj(os.path.join(FOLDER, 'cradle.obj'), transform(v, CRADLE), f, 'SS_V3_krybbe (cradle)')

    v, f = read_obj(cad(['SS_V3_brakett.obj']))
    write_obj(os.path.join(FOLDER, 'bracket.obj'), transform(v, BRACKET), f, 'SS_V3_brakett (bracket)')

    v, f = read_obj(cad(['Parallax Servo 900-00005.obj', 'Parallax_Servo_900-00005.obj']))
    body, horn = split_servo(v, f)
    v = transform(v, SERVO)
    write_obj(os.path.join(FOLDER, 'servo_body.obj'), v, body, 'Parallax 900-00005 body')
    write_obj(os.path.join(FOLDER, 'servo_horn.obj'), v, horn, 'Parallax 900-00005 horn')


def main():
    designs = sys.argv[1:] or ['v3', 'v5']
    if 'v3' in designs:
        convert_v3()
    if 'v5' in designs:
        convert_v5()


if __name__ == '__main__':
    main()