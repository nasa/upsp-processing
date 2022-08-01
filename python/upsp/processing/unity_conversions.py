import numpy as np
import os
from pathlib import Path

from . import io
from . import plot3d
from . import p3d_conversions


# NOTE: Unity uses a left hand coordinate system, and knows that obj files use
#       the right-hand coord convention. It will therefore negate the x values
#       of the obj file to convert the handedness open loading the obj mesh
#       into the 3D space. Between Unity and wind tunnel coord systems, it is
#       the z axis that is inverted; therefore to make Unity's representation
#       of the obj mesh align with the experimental setup, the obj file needs
#       both z and x axes to be negated.
def convert_to_unity_obj(p3d_file, obj_file, zones_map_file):
    grd = plot3d.read_p3d_grid(p3d_file)
    grd.write_zones_mapping(zones_map_file)
    tris = p3d_to_unity_obj_triangles(grd)

    v = tris["vertices"]
    f = tris["faces"]

    obj_path = os.path.dirname(obj_file)
    Path(obj_path).mkdir(parents=True, exist_ok=True)

    # Write obj file by listing vertices, vertex normals then faces
    with open(obj_file, "w+") as fp:
        fp.write("o Mesh_0\n")
        vList = map(lambda x: x + "\n", v)
        fp.writelines(vList)
        fp.write("usemtl None\n")
        fp.write("s off\n")
        fList = map(lambda x: x + "\n", f)
        fp.writelines(fList)


def p3d_to_unity_obj_triangles(grd):
    obj = p3d_conversions.p3d_to_obj_triangles(grd)

    vertices = obj["vertices"]
    unity_vertices = []
    for vertex in vertices:
        line = vertex.split(" ")
        # NOTE: the x- and z-values are negated to be Unity-compatible
        unity_line = (
            line[0]
            + " "
            + str(-1.0 * float(line[1]))
            + " "
            + line[2]
            + " "
            + str(-1.0 * float(line[3]))
        )
        unity_vertices.append(unity_line)

    obj["vertices"] = unity_vertices
    return obj


# NOTE: Unity uses a left hand coordinate system, so all data files that are
#       written for Unity consumption must negate the z axis in order to align
#       with the right handed experimental system setup
def convert_kulite_locations(kul_mat):
    unity_kul_mat = np.empty([len(kul_mat), len(kul_mat[0])], dtype=object)
    for i in range(0, len(kul_mat)):
        row = kul_mat[i]
        unity_z = str(-1.0 * float(row[3]))
        unity_k = str(-1.0 * float(row[8]))
        unity_row = [row[0], row[1], row[2], unity_z, row[4], row[5], row[6],
                     row[7], unity_k, row[9], row[10]]
        unity_kul_mat[i] = unity_row

    return unity_kul_mat
