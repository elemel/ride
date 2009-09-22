from __future__ import division

import models
from util import *

from euclid import *
from itertools import *
from xml.dom import minidom

def load_level(path):
    document = minidom.parse(path)
    svg_element = document.getElementsByTagName('svg')[0]
    width = float(svg_element.getAttribute('width'))
    height = float(svg_element.getAttribute('height'))
    description_element = document.getElementsByTagName('dc:description')[0]
    description = description_element.childNodes[0].nodeValue
    description_data = parse_style(description)
    world_width = float(description_data['width'])
    gravity = float(description_data.get('gravity', '10'))
    scale = world_width / width
    world_height = height * scale
    level_model = models.LevelModel()
    level_model.lower_bound = 0, 0
    level_model.upper_bound = world_width, world_height
    level_model.gravity = 0, -gravity
    transform = (Matrix3.new_scale(scale, -scale) *
                 Matrix3.new_translate(0, -height))
    for child_node in svg_element.childNodes:
        if child_node.nodeType == minidom.Node.ELEMENT_NODE:
            parse_element(child_node, transform, level_model)
    return level_model

def get_bodies_at_point(world, point):
    segment = b2.b2Segment()
    segment.p1 = tuple(point)
    segment.p2 = tuple(point)
    count, shapes = world.Raycast(segment, 1000, True, None)
    return list(set(s.GetBody() for s in shapes))

def get_bodies_at_line_segment(world, line_segment):
    bodies_1 = get_bodies_at_point(world, line_segment.p1)
    bodies_2 = get_bodies_at_point(world, line_segment.p2)
    if len(bodies_1) == 1 and len(bodies_2) == 2:
        bodies_2.remove(bodies_1[0])
    if len(bodies_1) == 2 and len(bodies_2) == 1:
        bodies_1.remove(bodies_2[0])
    return bodies_1[0], bodies_2[0]

def load_vehicle(path, level_model):
    document = minidom.parse(path)
    svg_element = document.getElementsByTagName('svg')[0]
    width = float(svg_element.getAttribute('width'))
    height = float(svg_element.getAttribute('height'))
    description_element = document.getElementsByTagName('dc:description')[0]
    description = description_element.childNodes[0].nodeValue
    world_width = float(parse_style(description)['width'])
    scale = world_width / width
    world_height = height * scale
    transform = (Matrix3.new_translate(*level_model.start) *
                 Matrix3.new_scale(scale, -scale) *
                 Matrix3.new_translate(0, -height))
    for child_node in svg_element.childNodes:
        if child_node.nodeType == minidom.Node.ELEMENT_NODE:
            parse_element(child_node, transform, level_model)

def parse_style(style):
    lines = (l.strip() for l in style.split(';'))
    pairs = (l.split(':') for l in lines if l)
    return dict((k.strip(), v.strip()) for k, v in pairs)

def parse_transform(transform_str):
    name, args = transform_str.split('(')
    name = name.strip()
    args = map(float, args.rstrip(')').split(','))
    if name == 'translate':
        return Matrix3.new_translate(*args)
    elif name == 'matrix':
        transform = Matrix3()
        transform[0:2] = args[0:2]
        transform[3:5] = args[2:4]
        transform[6:8] = args[4:6]
        return transform
    elif name == 'scale':
        return Matrix3.new_scale(*args)
    else:
        log('parse_transform(): unsupported SVG transform: %s' % name)
        return Matrix3.new_identity()

def parse_element(element, transform, level_model):
    transform_str = element.getAttribute('transform')
    if transform_str:
        transform = transform * parse_transform(transform_str)
    element_data = parse_element_data(element)
    if element_data.get('type') == 'revolute-joint':
        parse_revolute_joint_element(element, transform, level_model,
                                     element_data)
    elif element.nodeName == 'g':
        for child_node in element.childNodes:
            if child_node.nodeType == minidom.Node.ELEMENT_NODE:
                parse_element(child_node, transform, level_model)
    elif element.nodeName == 'path':
        if element.getAttribute('sodipodi:type') == 'arc':
            body_model = models.BodyModel()
            body_model.id = element.getAttribute('id')
            level_model.body_models.append(body_model)
            parse_circle_element(element, transform, level_model, body_model)
        else:
            parse_path_element(element, transform, level_model)
    elif element.nodeName == 'rect':
        body_model = models.BodyModel()
        body_model.id = element.getAttribute('id')
        level_model.body_models.append(body_model)
        parse_rect_element(element, transform, level_model, body_model)
    elif element.nodeName not in ('sodipodi:namedview', 'defs', 'metadata'):
        log('parse_element(): unsupported SVG element: %s' % element.nodeName)

def parse_element_data(element):
    for child_node in element.childNodes:
        if (child_node.nodeType == minidom.Node.ELEMENT_NODE and
            child_node.nodeName == 'desc'):
            return parse_style(child_node.childNodes[0].nodeValue)
    return {}

def parse_line_segment_element(element, transform):
    path = element.getAttribute('d').replace(',', ' ').strip()
    points = path.lstrip('M').split('L')
    assert len(points) == 2
    points = [Point2(*map(float, p.split())) for p in points]
    return transform * LineSegment2(*points)

def parse_polygon(path):
    path = path.strip().lstrip('M')
    closed = path.endswith('z')
    vertices = [Point2(*map(float, p.replace(',', ' ').split()))
                for p in path.rstrip('z').split('L')]
    return vertices, closed

def parse_revolute_joint_element(element, transform, level_model,
                                 element_data):
    x = float(element.getAttribute('sodipodi:cx'))
    y = float(element.getAttribute('sodipodi:cy'))
    revolute_joint_model = models.RevoluteJointModel()
    revolute_joint_model.anchor = transform * Point2(x, y)
    level_model.joint_models.append(revolute_joint_model)

def parse_distance_joint_element(element, transform, level_model):
    line_segment = parse_line_segment_element(element, transform)
    distance_joint_model = models.DistanceJointModel()
    distance_joint_model.anchor_1 = line_segment.p1
    distance_joint_model.anchor_2 = line_segment.p2
    level_model.joint_models.append(distance_joint_model)

def parse_prismatic_joint_element(element, transform, level_model):
    line_segment = parse_line_segment_element(element, transform)
    prismatic_joint_model = models.PrismaticJointModel()
    prismatic_joint_model.anchor_1 = line_segment.p1
    prismatic_joint_model.anchor_2 = line_segment.p2
    level_model.joint_models.append(prismatic_joint_model)

def parse_spring_element(element, transform, level_model, element_data):
    line_segment = parse_line_segment_element(element, transform)
    spring_model = models.SpringModel()
    spring_model.anchor_1 = line_segment.p1
    spring_model.anchor_2 = line_segment.p2
    spring_model.spring_constant = float(element_data.get('spring-constant', '1'))
    spring_model.damping = float(element_data.get('damping', '0'))
    level_model.joint_models.append(spring_model)

def parse_path_element(element, transform, level_model):
    element_data = parse_element_data(element)
    if element_data.get('type') == 'start':
        x = float(element.getAttribute('sodipodi:cx'))
        y = float(element.getAttribute('sodipodi:cy'))
        level_model.start = transform * Point2(x, y)
    elif element_data.get('type') == 'goal':
        x = float(element.getAttribute('sodipodi:cx'))
        y = float(element.getAttribute('sodipodi:cy'))
        level_model.goal = transform * Point2(x, y)
    elif element_data.get('type') == 'distance-joint':
        parse_distance_joint_element(element, transform, level_model)
    elif element_data.get('type') == 'prismatic-joint':
        parse_prismatic_joint_element(element, transform, level_model)
    elif element_data.get('type') == 'spring':
        parse_spring_element(element, transform, level_model, element_data)
    elif element_data.get('type') == 'motor':
        x = float(element.getAttribute('sodipodi:cx'))
        y = float(element.getAttribute('sodipodi:cy'))
        motor_model = models.MotorModel()
        motor_model.anchor = transform * Point2(x, y)
        motor_model.torque = float(element_data.get('torque', '1'))
        motor_model.damping = float(element_data.get('damping', '0'))
        motor_model.clockwise_key = element_data.get('clockwise-key')
        motor_model.counter_clockwise_key = element_data.get('counter-clockwise-key')
        level_model.joint_models.append(motor_model)
    elif element_data.get('type') == 'camera':
        x = float(element.getAttribute('sodipodi:cx'))
        y = float(element.getAttribute('sodipodi:cy'))
        camera_model = models.CameraModel()
        camera_model.anchor = transform * Point2(x, y)
        level_model.joint_models.append(camera_model)
    else:
        vertices, closed = parse_polygon(element.getAttribute('d'))
        vertices = [transform * v for v in vertices]
        body_model = models.BodyModel()
        body_model.id = element.getAttribute('id')
        radius = 0.2
        kwargs = dict(density=float(element_data.get('density', '0')),
                      friction=float(element_data.get('friction', '0.5')),
                      restitution=float(element_data.get('restitution', '0.5')),
                      group_index=int(element_data.get('group-index', '0')))
        for p1, p2 in izip(vertices[:-1], vertices[1:]):
            polygon_model = models.PolygonModel(**kwargs)
            v = (p2 - p1).cross()
            v.normalize()
            v *= radius
            polygon_model.vertices = [p1 + v, p2 + v, p2 - v, p1 - v]
            body_model.shape_models.append(polygon_model)
            for center in p1, p2:
                circle_model = models.CircleModel(center=center, radius=radius, **kwargs)
                body_model.shape_models.append(circle_model)
        level_model.body_models.append(body_model)

def parse_circle_element(element, transform, level_model, body_model):
    label = element.getAttribute('inkscape:label')
    element_data = parse_element_data(element)

    cx = float(element.getAttribute('sodipodi:cx'))
    cy = float(element.getAttribute('sodipodi:cy'))
    rx = float(element.getAttribute('sodipodi:rx'))
    ry = float(element.getAttribute('sodipodi:ry'))

    circle_model = models.CircleModel()
    circle_model.center = transform * Point2(cx, cy)
    circle_model.radius = abs(transform * Vector2((rx + ry) / 2, 0))
    circle_model.density = float(element_data.get('density', '0'))
    circle_model.friction = float(element_data.get('friction', '0.5'))
    circle_model.restitution = float(element_data.get('restitution', '0.5'))
    circle_model.group_index = int(element_data.get('group-index', '0'))
    body_model.shape_models.append(circle_model)

def parse_rect_element(element, transform, level_model, body_model):
    label = element.getAttribute('inkscape:label')
    element_data = parse_element_data(element)
    x = float(element.getAttribute('x'))
    y = float(element.getAttribute('y'))
    width = float(element.getAttribute('width'))
    height = float(element.getAttribute('height'))
    vertices = [Point2(x, y),
                Point2(x, y + height),
                Point2(x + width, y + height),
                Point2(x + width, y)]

    polygon_model = models.PolygonModel()
    polygon_model.vertices = [tuple(transform * v) for v in vertices]
    polygon_model.density = float(element_data.get('density', '0'))
    polygon_model.friction = float(element_data.get('friction', '0.5'))
    polygon_model.restitution = float(element_data.get('restitution', '0.5'))
    polygon_model.group_index = int(element_data.get('group-index', '0'))
    body_model.shape_models.append(polygon_model)
