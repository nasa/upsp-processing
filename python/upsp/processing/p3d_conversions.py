import base64
import json
import struct
import os
import subprocess

from pathlib import Path

from . import plot3d


def convert_to_obj(p3d_file, obj_file, zones_map_file):
    grd = plot3d.read_p3d_grid(p3d_file)
    grd.write_zones_mapping(zones_map_file)
    tris = p3d_to_obj_triangles(grd)

    v = tris["vertices"]
    f = tris["faces"]

    obj_path = os.path.dirname(obj_file)
    Path(obj_path).mkdir(parents=True, exist_ok=True)

    # Write obj file by listing vertices, vertex normals then faces
    fp = open(obj_file, "w+")
    fp.write("o Mesh_0\n")
    vList = map(lambda x: x + "\n", v)
    fp.writelines(vList)
    fp.write("usemtl None\n")
    fp.write("s off\n")
    fList = map(lambda x: x + "\n", f)
    fp.writelines(fList)

    subprocess.call(["chmod", "750", obj_file])


def p3d_to_obj_triangles(grd):
    vertices = []
    faces = []
    idx0 = 0
    for imax, jmax, kmax in grd.sz:

        for ii in range(imax * jmax):
            line = (
                "v "
                + str(grd.x[idx0 + ii])
                + " "
                + str(grd.y[idx0 + ii])
                + " "
                + str(grd.z[idx0 + ii])
            )
            vertices.append(line)

        for ii in range(imax - 1):
            for jj in range(jmax - 1):
                p0 = idx0 + jj * imax + ii
                p1 = idx0 + jj * imax + ii + 1
                p2 = idx0 + (jj + 1) * imax + ii + 1
                p3 = idx0 + (jj + 1) * imax + ii

                # obj files assume counter-clockwise vertex order;
                # assume normals have identical index to vertex
                f0 = "f " + str(p0 + 1) + " " + str(p1 + 1) + " " + str(p2 + 1)
                f1 = "f " + str(p0 + 1) + " " + str(p2 + 1) + " " + str(p3 + 1)

                faces.append(f0)
                faces.append(f1)

        idx0 += imax * jmax
    return {"vertices": vertices, "faces": faces}


def convert_to_gltf(p3d_file, gltf_file, zones_mapping_file):

    grd = plot3d.read_p3d_grid(p3d_file)
    grd.write_zones_mapping(zones_mapping_file)
    tris = p3d_to_gltf_triangles(grd)

    v = tris["vertices"]
    vmax = [float(max(v[0::3])), float(max(v[1::3])), float(max(v[2::3]))]
    vmin = [float(min(v[0::3])), float(min(v[1::3])), float(min(v[2::3]))]

    f = tris["indices"]
    fmax = int(max(f))
    fmin = int(min(f))

    npad = (len(f) * 4) % 4
    fbuff = struct.pack("<" + len(f) * "I" + npad * "x", *f)
    vbuff = struct.pack("<" + len(v) * "f", *v)

    data = fbuff + vbuff
    data_uri = "data:application/octet-stream;base64," + base64.b64encode(data).decode(
        "utf-8"
    )

    buff = {"uri": data_uri, "byteLength": len(data)}

    SCALAR = "SCALAR"
    VEC2 = "VEC2"  # noqa
    VEC3 = "VEC3"
    VEC4 = "VEC4"  # noqa
    MAT2 = "MAT2"  # noqa
    MAT3 = "MAT3"  # noqa
    MAT4 = "MAT4"  # noqa

    BYTE = 5120  # noqa
    UNSIGNED_BYTE = 5121  # noqa
    SHORT = 5122  # noqa
    UNSIGNED_SHORT = 5123  # noqa
    UNSIGNED_INT = 5125
    FLOAT = 5126

    # MESH PRIMITIVE MODES
    POINTS = 0  # noqa
    LINES = 1  # noqa
    LINE_LOOP = 2  # noqa
    LINE_STRIP = 3  # noqa
    TRIANGLES = 4
    TRIANGLE_STRIP = 5  # noqa
    TRIANGLE_FAN = 6  # noqa

    ARRAY_BUFFER = 34962  # eg vertex data
    ELEMENT_ARRAY_BUFFER = 34963  # eg index data

    indices_view = {
        "buffer": 0,
        "byteOffset": 0,
        "byteLength": len(fbuff),
        "target": ELEMENT_ARRAY_BUFFER,
    }

    vertices_view = {
        "buffer": 0,
        "byteOffset": len(fbuff),
        "byteLength": len(vbuff),
        "target": ARRAY_BUFFER,
    }

    indices_accessor = {
        "bufferView": 0,
        "byteOffset": 0,
        "componentType": UNSIGNED_INT,
        "count": int(len(f)),
        "type": SCALAR,
        "max": [fmax],
        "min": [fmin],
    }

    vertices_accessor = {
        "bufferView": 1,
        "byteOffset": 0,
        "componentType": FLOAT,
        "count": int(len(v) / 3),
        "type": VEC3,
        "max": vmax,
        "min": vmin,
    }

    material = {
        "pbrMetallicRoughness": {
            "baseColorTexture": {"index": 0},
            "metallicFactor": 0.5,
            "roughnessFactor": 0.1,
            "alphaMode": "OPAQUE",
            "alphaCutoff": 0.5,
        }
    }

    texture = {"sampler": 0, "source": 0}

    sampler = {"magFilter": 9729, "minFilter": 9987, "wrapS": 33648, "wrapT": 33648}

    primitive = {
        "attributes": {
            "POSITION": 1,
        },
        "indices": 0,
        "mode": TRIANGLES,
    }

    gltf = {
        "asset": {"version": "2.0"},
        "scenes": [{"nodes": [0]}],
        "nodes": [{"mesh": 0}],
        "meshes": [{"primitives": [primitive]}],
        "buffers": [buff],
        "bufferViews": [indices_view, vertices_view],
        "accessors": [indices_accessor, vertices_accessor],
        "textures": [texture],
        "samplers": [sampler],
        "materials": [material],
    }

    gltf_path = os.path.dirname(gltf_file)
    Path(gltf_path).mkdir(parents=True, exist_ok=True)

    with open(gltf_file, "w+") as fp:
        json.dump(gltf, fp, indent=True)
    subprocess.call(["chmod", "750", gltf_file])


def p3d_to_gltf_triangles(grd):
    vertices = []
    indices = []
    idx0 = 0
    for imax, jmax, kmax in grd.sz:
        for ii in range(imax * jmax):
            vertices.append(grd.x[idx0 + ii])
            vertices.append(grd.y[idx0 + ii])
            vertices.append(grd.z[idx0 + ii])

        for ii in range(imax - 1):
            for jj in range(jmax - 1):
                p0 = idx0 + jj * imax + ii
                p1 = idx0 + jj * imax + ii + 1
                p2 = idx0 + (jj + 1) * imax + ii + 1
                p3 = idx0 + (jj + 1) * imax + ii
                t0 = [p0, p1, p2]
                t1 = [p0, p2, p3]
                indices.extend(t0)
                indices.extend(t1)
        idx0 += imax * jmax
    return {"vertices": vertices, "indices": indices}
