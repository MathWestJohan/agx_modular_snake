import os

FOLDER = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'environment', 'assets', 'ssv3')
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


def cad(name_options):
    for name in name_options:
        path = os.path.join(FOLDER, name)
        if os.path.exists(path):
            return path
    raise FileNotFoundError('none of {} in {}'.format(name_options, FOLDER))


def main():
    v, f = read_obj(cad(['SS_V3_krybbe.obj']))
    write_obj(os.path.join(FOLDER, 'cradle.obj'), transform(v, CRADLE), f, 'SS_V3_krybbe (cradle)')

    v, f = read_obj(cad(['SS_V3_brakett.obj']))
    write_obj(os.path.join(FOLDER, 'bracket.obj'), transform(v, BRACKET), f, 'SS_V3_brakett (bracket)')

    v, f = read_obj(cad(['Parallax Servo 900-00005.obj', 'Parallax_Servo_900-00005.obj']))
    body, horn = split_servo(v, f)
    v = transform(v, SERVO)
    write_obj(os.path.join(FOLDER, 'servo_body.obj'), v, body, 'Parallax 900-00005 body')
    write_obj(os.path.join(FOLDER, 'servo_horn.obj'), v, horn, 'Parallax 900-00005 horn')


if __name__ == '__main__':
    main()