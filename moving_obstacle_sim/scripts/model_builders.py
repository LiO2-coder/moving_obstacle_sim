#!/usr/bin/env python3

import copy
import math
import xml.etree.ElementTree as ET


def _to_bool_text(value):
    return "true" if bool(value) else "false"


def _to_float(value, default=0.0):
    try:
        return float(value)
    except (TypeError, ValueError):
        return float(default)


def _to_list(value, length, default):
    if not isinstance(value, (list, tuple)):
        return list(default)
    out = list(default)
    for i in range(min(length, len(value))):
        out[i] = _to_float(value[i], default[i])
    return out


def _normalize_shape(shape_cfg):
    shape = copy.deepcopy(shape_cfg) if isinstance(shape_cfg, dict) else {}
    shape_type = str(shape.get("type", "box")).strip().lower()
    if shape_type not in ("box", "sphere", "cylinder"):
        shape_type = "box"

    shape["type"] = shape_type
    shape["mass"] = max(_to_float(shape.get("mass", 1.0), 1.0), 1e-6)
    shape["box_size"] = _to_list(shape.get("box_size", [0.5, 0.5, 0.5]), 3, [0.5, 0.5, 0.5])
    shape["radius"] = max(_to_float(shape.get("radius", 0.25), 0.25), 1e-6)
    shape["length"] = max(_to_float(shape.get("length", 0.6), 0.6), 1e-6)
    shape["color_rgba"] = _to_list(shape.get("color_rgba", [0.7, 0.7, 0.7, 1.0]), 4, [0.7, 0.7, 0.7, 1.0])

    if shape_type == "box":
        w, d, h = shape["box_size"]
        shape["box_size"] = [max(w, 1e-6), max(d, 1e-6), max(h, 1e-6)]
    return shape


def _normalize_pose(pose_cfg, shape):
    pose = copy.deepcopy(pose_cfg) if isinstance(pose_cfg, dict) else {}
    xyz = _to_list(pose.get("xyz", [0.0, 0.0, 0.0]), 3, [0.0, 0.0, 0.0])
    rpy = _to_list(pose.get("rpy", [0.0, 0.0, 0.0]), 3, [0.0, 0.0, 0.0])

    if abs(xyz[2]) < 1e-9:
        if shape["type"] == "box":
            xyz[2] = shape["box_size"][2] * 0.5
        elif shape["type"] == "sphere":
            xyz[2] = shape["radius"]
        else:
            xyz[2] = shape["length"] * 0.5

    return {"xyz": xyz, "rpy": rpy}


def _normalize_motion(motion_cfg, pose):
    motion = copy.deepcopy(motion_cfg) if isinstance(motion_cfg, dict) else {}

    motion["enabled"] = bool(motion.get("enabled", True))
    motion_type = str(motion.get("type", "static")).strip().lower()
    if motion_type not in ("static", "line", "sine", "circle", "ellipse", "polygon"):
        motion_type = "static"
    motion["type"] = motion_type

    mode = str(motion.get("mode", "ping_pong")).strip().lower()
    if mode not in ("loop", "ping_pong", "once"):
        mode = "ping_pong"
    motion["mode"] = mode

    motion["speed"] = _to_float(motion.get("speed", 0.5), 0.5)
    motion["align_yaw"] = bool(motion.get("align_yaw", False))
    motion["z_lock"] = bool(motion.get("z_lock", True))

    bounds = motion.get("bounds", {}) if isinstance(motion.get("bounds", {}), dict) else {}
    motion["bounds"] = {
        "enabled": bool(bounds.get("enabled", False)),
        "x_min": _to_float(bounds.get("x_min", -10.0), -10.0),
        "x_max": _to_float(bounds.get("x_max", 10.0), 10.0),
        "y_min": _to_float(bounds.get("y_min", -10.0), -10.0),
        "y_max": _to_float(bounds.get("y_max", 10.0), 10.0),
    }

    px, py = pose["xyz"][0], pose["xyz"][1]

    line = motion.get("line", {}) if isinstance(motion.get("line", {}), dict) else {}
    motion["line"] = {
        "p0": _to_list(line.get("p0", [px - 1.0, py]), 2, [px - 1.0, py]),
        "p1": _to_list(line.get("p1", [px + 1.0, py]), 2, [px + 1.0, py]),
    }

    sine = motion.get("sine", {}) if isinstance(motion.get("sine", {}), dict) else {}
    axis = str(sine.get("axis", "x")).strip().lower()
    if axis not in ("x", "y"):
        axis = "x"
    motion["sine"] = {
        "axis": axis,
        "x_min": _to_float(sine.get("x_min", px - 1.0), px - 1.0),
        "x_max": _to_float(sine.get("x_max", px + 1.0), px + 1.0),
        "y_min": _to_float(sine.get("y_min", py - 1.0), py - 1.0),
        "y_max": _to_float(sine.get("y_max", py + 1.0), py + 1.0),
        "center": _to_float(sine.get("center", py if axis == "x" else px), py if axis == "x" else px),
        "amplitude": _to_float(sine.get("amplitude", 0.5), 0.5),
        "wavelength": _to_float(sine.get("wavelength", 2.0), 2.0),
        "phase": _to_float(sine.get("phase", 0.0), 0.0),
    }

    circle = motion.get("circle", {}) if isinstance(motion.get("circle", {}), dict) else {}
    motion["circle"] = {
        "center": _to_list(circle.get("center", [px, py]), 2, [px, py]),
        "radius": _to_float(circle.get("radius", 1.0), 1.0),
        "clockwise": bool(circle.get("clockwise", False)),
        "theta0": _to_float(circle.get("theta0", 0.0), 0.0),
    }

    ellipse = motion.get("ellipse", {}) if isinstance(motion.get("ellipse", {}), dict) else {}
    motion["ellipse"] = {
        "center": _to_list(ellipse.get("center", [px, py]), 2, [px, py]),
        "a": _to_float(ellipse.get("a", 1.5), 1.5),
        "b": _to_float(ellipse.get("b", 1.0), 1.0),
        "yaw": _to_float(ellipse.get("yaw", 0.0), 0.0),
        "clockwise": bool(ellipse.get("clockwise", False)),
        "theta0": _to_float(ellipse.get("theta0", 0.0), 0.0),
    }

    polygon = motion.get("polygon", {}) if isinstance(motion.get("polygon", {}), dict) else {}
    raw_points = polygon.get("points", [[px - 1.0, py - 1.0], [px + 1.0, py - 1.0], [px + 1.0, py + 1.0]])
    points = []
    if isinstance(raw_points, (list, tuple)):
        for point in raw_points:
            points.append(_to_list(point, 2, [px, py]))
    motion["polygon"] = {
        "points": points,
        "close_loop": bool(polygon.get("close_loop", True)),
    }

    return motion


def _inertia_from_shape(shape):
    m = shape["mass"]
    if shape["type"] == "box":
        w, d, h = shape["box_size"]
        ixx = (m / 12.0) * (d * d + h * h)
        iyy = (m / 12.0) * (w * w + h * h)
        izz = (m / 12.0) * (w * w + d * d)
    elif shape["type"] == "sphere":
        r = shape["radius"]
        i = (2.0 / 5.0) * m * r * r
        ixx = iyy = izz = i
    else:
        r = shape["radius"]
        l = shape["length"]
        ixx = (m / 12.0) * (3.0 * r * r + l * l)
        iyy = ixx
        izz = 0.5 * m * r * r

    return {
        "ixx": ixx,
        "iyy": iyy,
        "izz": izz,
        "ixy": 0.0,
        "ixz": 0.0,
        "iyz": 0.0,
    }


def _build_geometry(parent, shape):
    geometry = ET.SubElement(parent, "geometry")
    if shape["type"] == "box":
        box = ET.SubElement(geometry, "box")
        ET.SubElement(box, "size").text = "{:.6f} {:.6f} {:.6f}".format(*shape["box_size"])
    elif shape["type"] == "sphere":
        sphere = ET.SubElement(geometry, "sphere")
        ET.SubElement(sphere, "radius").text = "{:.6f}".format(shape["radius"])
    else:
        cylinder = ET.SubElement(geometry, "cylinder")
        ET.SubElement(cylinder, "radius").text = "{:.6f}".format(shape["radius"])
        ET.SubElement(cylinder, "length").text = "{:.6f}".format(shape["length"])


def _serialize_polygon_points(points):
    serialized = []
    for p in points:
        serialized.append("{:.6f} {:.6f}".format(_to_float(p[0], 0.0), _to_float(p[1], 0.0)))
    return ";".join(serialized)


def _add_plugin(model, motion, plugin_filename):
    plugin = ET.SubElement(
        model,
        "plugin",
        {
            "name": "moving_obstacle_controller",
            "filename": plugin_filename,
        },
    )

    ET.SubElement(plugin, "enabled").text = _to_bool_text(motion["enabled"])
    ET.SubElement(plugin, "type").text = str(motion["type"])
    ET.SubElement(plugin, "speed").text = "{:.6f}".format(motion["speed"])
    ET.SubElement(plugin, "mode").text = str(motion["mode"])
    ET.SubElement(plugin, "align_yaw").text = _to_bool_text(motion["align_yaw"])
    ET.SubElement(plugin, "z_lock").text = _to_bool_text(motion["z_lock"])

    bounds = motion["bounds"]
    ET.SubElement(plugin, "bounds_enabled").text = _to_bool_text(bounds["enabled"])
    ET.SubElement(plugin, "x_min").text = "{:.6f}".format(bounds["x_min"])
    ET.SubElement(plugin, "x_max").text = "{:.6f}".format(bounds["x_max"])
    ET.SubElement(plugin, "y_min").text = "{:.6f}".format(bounds["y_min"])
    ET.SubElement(plugin, "y_max").text = "{:.6f}".format(bounds["y_max"])

    line = motion["line"]
    ET.SubElement(plugin, "line_p0").text = "{:.6f} {:.6f}".format(line["p0"][0], line["p0"][1])
    ET.SubElement(plugin, "line_p1").text = "{:.6f} {:.6f}".format(line["p1"][0], line["p1"][1])

    sine = motion["sine"]
    ET.SubElement(plugin, "sine_axis").text = sine["axis"]
    ET.SubElement(plugin, "sine_x_min").text = "{:.6f}".format(sine["x_min"])
    ET.SubElement(plugin, "sine_x_max").text = "{:.6f}".format(sine["x_max"])
    ET.SubElement(plugin, "sine_y_min").text = "{:.6f}".format(sine["y_min"])
    ET.SubElement(plugin, "sine_y_max").text = "{:.6f}".format(sine["y_max"])
    ET.SubElement(plugin, "sine_center").text = "{:.6f}".format(sine["center"])
    ET.SubElement(plugin, "sine_amplitude").text = "{:.6f}".format(sine["amplitude"])
    ET.SubElement(plugin, "sine_wavelength").text = "{:.6f}".format(sine["wavelength"])
    ET.SubElement(plugin, "sine_phase").text = "{:.6f}".format(sine["phase"])

    circle = motion["circle"]
    ET.SubElement(plugin, "circle_center").text = "{:.6f} {:.6f}".format(circle["center"][0], circle["center"][1])
    ET.SubElement(plugin, "circle_radius").text = "{:.6f}".format(circle["radius"])
    ET.SubElement(plugin, "circle_clockwise").text = _to_bool_text(circle["clockwise"])
    ET.SubElement(plugin, "circle_theta0").text = "{:.6f}".format(circle["theta0"])

    ellipse = motion["ellipse"]
    ET.SubElement(plugin, "ellipse_center").text = "{:.6f} {:.6f}".format(ellipse["center"][0], ellipse["center"][1])
    ET.SubElement(plugin, "ellipse_a").text = "{:.6f}".format(ellipse["a"])
    ET.SubElement(plugin, "ellipse_b").text = "{:.6f}".format(ellipse["b"])
    ET.SubElement(plugin, "ellipse_yaw").text = "{:.6f}".format(ellipse["yaw"])
    ET.SubElement(plugin, "ellipse_clockwise").text = _to_bool_text(ellipse["clockwise"])
    ET.SubElement(plugin, "ellipse_theta0").text = "{:.6f}".format(ellipse["theta0"])

    polygon = motion["polygon"]
    ET.SubElement(plugin, "polygon_points").text = _serialize_polygon_points(polygon["points"])
    ET.SubElement(plugin, "polygon_close_loop").text = _to_bool_text(polygon["close_loop"])


def build_model_sdf(obstacle, obstacle_index=0, plugin_filename="libmoving_obstacle_plugin.so"):
    cfg = copy.deepcopy(obstacle) if isinstance(obstacle, dict) else {}
    model_name = str(cfg.get("name", "moving_obstacle_{}".format(obstacle_index))).strip() or "moving_obstacle_{}".format(obstacle_index)
    enabled = bool(cfg.get("enabled", True))

    shape = _normalize_shape(cfg.get("shape", {}))
    pose = _normalize_pose(cfg.get("pose", {}), shape)
    motion = _normalize_motion(cfg.get("motion", {}), pose)

    if not enabled:
        motion["enabled"] = False
        motion["type"] = "static"

    is_static = (not motion["enabled"]) or motion["type"] == "static"
    inertia = _inertia_from_shape(shape)

    sdf = ET.Element("sdf", {"version": "1.6"})
    model = ET.SubElement(sdf, "model", {"name": model_name})
    ET.SubElement(model, "pose").text = "{:.6f} {:.6f} {:.6f} {:.6f} {:.6f} {:.6f}".format(
        pose["xyz"][0],
        pose["xyz"][1],
        pose["xyz"][2],
        pose["rpy"][0],
        pose["rpy"][1],
        pose["rpy"][2],
    )
    ET.SubElement(model, "static").text = _to_bool_text(is_static)
    ET.SubElement(model, "allow_auto_disable").text = "false"

    link = ET.SubElement(model, "link", {"name": "body"})
    ET.SubElement(link, "gravity").text = _to_bool_text(False)
    ET.SubElement(link, "kinematic").text = _to_bool_text(not is_static)
    ET.SubElement(link, "self_collide").text = "false"

    inertial = ET.SubElement(link, "inertial")
    ET.SubElement(inertial, "mass").text = "{:.6f}".format(shape["mass"])
    inertia_elem = ET.SubElement(inertial, "inertia")
    for key in ("ixx", "ixy", "ixz", "iyy", "iyz", "izz"):
        ET.SubElement(inertia_elem, key).text = "{:.6f}".format(inertia[key])

    collision = ET.SubElement(link, "collision", {"name": "collision"})
    _build_geometry(collision, shape)

    visual = ET.SubElement(link, "visual", {"name": "visual"})
    _build_geometry(visual, shape)
    material = ET.SubElement(visual, "material")
    r, g, b, a = shape["color_rgba"]
    ET.SubElement(material, "ambient").text = "{:.6f} {:.6f} {:.6f} {:.6f}".format(r, g, b, a)
    ET.SubElement(material, "diffuse").text = "{:.6f} {:.6f} {:.6f} {:.6f}".format(r, g, b, a)
    ET.SubElement(material, "specular").text = "0.100000 0.100000 0.100000 1.000000"

    _add_plugin(model, motion, plugin_filename)
    return ET.tostring(sdf, encoding="unicode")
