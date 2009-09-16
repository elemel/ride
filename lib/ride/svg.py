from __future__ import division

import models

import euclid
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
    transform = (euclid.Matrix3.new_scale(scale, -scale) *
                 euclid.Matrix3.new_translate(0, -height))
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
    transform = (euclid.Matrix3.new_translate(*level_model.start) *
                 euclid.Matrix3.new_scale(scale, -scale) *
                 euclid.Matrix3.new_translate(0, -height))
    for child_node in svg_element.childNodes:
        if child_node.nodeType == minidom.Node.ELEMENT_NODE:
            parse_element(child_node, transform, level_model)

    """
    for line_segment in state.distance_joints:
        bodies = get_bodies_at_line_segment(state.level.world, line_segment)
        joint_def = b2.b2DistanceJointDef()
        joint_def.Initialize(bodies[0], bodies[1], tuple(line_segment.p1),
                             tuple(line_segment.p2))
        state.level.world.CreateJoint(joint_def)
    for line_segment, element_data in state.springs:
        bodies = get_bodies_at_line_segment(state.level.world, line_segment)
        spring_constant = float(element_data.get('spring-constant', '1'))
        damping = float(element_data.get('damping', '0'))
        max_force = float(element_data.get('max-force', '1'))
        spring = Spring(bodies[0], bodies[1], b2.b2Vec2(*line_segment.p1),
                        b2.b2Vec2(*line_segment.p2),
                        spring_constant=spring_constant,
                        damping=damping,
                        max_force=max_force)
        state.level.add_spring(spring)
    """

def parse_style(style):
    lines = (l.strip() for l in style.split(';'))
    pairs = (l.split(':') for l in lines if l)
    return dict((k.strip(), v.strip()) for k, v in pairs)

def parse_transform(transform_str):
    name, args = transform_str.split('(')
    name = name.strip()
    args = map(float, args.rstrip(')').split(','))
    if name == 'translate':
        return euclid.Matrix3.new_translate(*args)
    elif name == 'matrix':
        transform = euclid.Matrix3()
        transform[0:2] = args[0:2]
        transform[3:5] = args[2:4]
        transform[6:8] = args[4:6]
        return transform
    else:
        log('parse_transform(): unsupported SVG transform: ' + transform_str)
        return euclid.Matrix3.new_identity()

def parse_element(element, transform, level_model):
    transform_str = element.getAttribute('transform')
    if transform_str:
        transform = transform * parse_transform(transform_str)
    if element.nodeName == 'g':
        for child_node in element.childNodes:
            if child_node.nodeType == minidom.Node.ELEMENT_NODE:
                parse_element(child_node, transform, level_model)
    elif element.nodeName == 'path':
        if element.getAttribute('sodipodi:type') == 'arc':
            body = models.BodyModel()
            level_model.body_models.add(body)
            parse_circle_element(element, transform, level_model, body)
        else:
            parse_path_element(element, transform, level_model)
    elif element.nodeName == 'rect':
        body_model = models.BodyModel()
        level_model.body_models.add(body_model)
        parse_rect_element(element, transform, level_model, body_model)
    elif element.nodeName not in ('sodipodi:namedview', 'defs', 'metadata'):
        log('parse_element(): unsupported SVG element: ' + str(element))

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
    points = [euclid.Point2(*map(float, p.split())) for p in points]
    return transform * euclid.LineSegment2(*points)

def parse_distance_joint_element(element, transform, level_model):
    line_segment = parse_line_segment_element(element, transform)
    distance_joint_model = models.DistanceJointModel()
    distance_joint_model.anchor_1 = line_segment.p1
    distance_joint_model.anchor_2 = line_segment.p2
    level_model.joint_models.add(distance_joint_model)

def parse_spring_element(element, transform, level_model, element_data):
    line_segment = parse_line_segment_element(element, transform)
    spring_model = models.SpringModel()
    spring_model.anchor_1 = line_segment.p1
    spring_model.anchor_2 = line_segment.p2
    spring_model.spring_constant = float(element_data.get('spring-constant', '1'))
    spring_model.damping_constant = float(element_data.get('damping-constant', '0'))
    level_model.joint_models.add(spring_model)

def parse_path_element(element, transform, level_model):
    element_data = parse_element_data(element)
    if element_data.get('type') == 'start':
        x = float(element.getAttribute('sodipodi:cx'))
        y = float(element.getAttribute('sodipodi:cy'))
        level_model.start = transform * euclid.Point2(x, y)
    elif element_data.get('type') == 'goal':
        x = float(element.getAttribute('sodipodi:cx'))
        y = float(element.getAttribute('sodipodi:cy'))
        level_model.goal = transform * euclid.Point2(x, y)
    elif element_data.get('type') == 'distance-joint':
        parse_distance_joint_element(element, transform, level_model)
    elif element_data.get('type') == 'spring':
        parse_spring_element(element, transform, level_model, element_data)

def parse_circle_element(element, transform, level_model, body_model):
    label = element.getAttribute('inkscape:label')
    element_data = parse_element_data(element)

    cx = float(element.getAttribute('sodipodi:cx'))
    cy = float(element.getAttribute('sodipodi:cy'))
    rx = float(element.getAttribute('sodipodi:rx'))
    ry = float(element.getAttribute('sodipodi:ry'))

    circle_model = models.CircleModel()
    circle_model.center = transform * euclid.Point2(cx, cy)
    circle_model.radius = abs(transform * euclid.Vector2((rx + ry) / 2, 0))
    circle_model.density = float(element_data.get('density', '0'))
    circle_model.friction = float(element_data.get('friction', '0.5'))
    circle_model.restitution = float(element_data.get('restitution', '0.5'))
    body_model.shape_models.add(circle_model)

def parse_rect_element(element, transform, level_model, body_model):
    label = element.getAttribute('inkscape:label')
    element_data = parse_element_data(element)
    x = float(element.getAttribute('x'))
    y = float(element.getAttribute('y'))
    width = float(element.getAttribute('width'))
    height = float(element.getAttribute('height'))
    vertices = [euclid.Point2(x, y),
                euclid.Point2(x, y + height),
                euclid.Point2(x + width, y + height),
                euclid.Point2(x + width, y)]

    polygon_model = models.PolygonModel()
    polygon_model.vertices = [tuple(transform * v) for v in vertices]
    polygon_model.density = float(element_data.get('density', '0'))
    polygon_model.friction = float(element_data.get('friction', '0.5'))
    polygon_model.restitution = float(element_data.get('restitution', '0.5'))
    body_model.shape_models.add(polygon_model)
