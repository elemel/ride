from __future__ import division

from game import *

import euclid
from xml.dom import minidom

def load_level(path):
    document = minidom.parse(path)
    svg_element = document.getElementsByTagName('svg')[0]
    width = float(svg_element.getAttribute('width'))
    height = float(svg_element.getAttribute('height'))
    description_element = document.getElementsByTagName('dc:description')[0]
    description = description_element.childNodes[0].nodeValue
    world_width = float(parse_style(description)['width'])
    scale = world_width / width
    world_height = height * scale
    level = Level((0, 0), (width, height))
    transform = (euclid.Matrix3.new_scale(scale, -scale) *
                 euclid.Matrix3.new_translate(0, -height))
    for child_node in svg_element.childNodes:
        if child_node.nodeType == minidom.Node.ELEMENT_NODE:
            parse_element(child_node, transform, level)
    return level

def load_vehicle(path, level):
    document = minidom.parse(path)
    svg_element = document.getElementsByTagName('svg')[0]
    width = float(svg_element.getAttribute('width'))
    height = float(svg_element.getAttribute('height'))
    description_element = document.getElementsByTagName('dc:description')[0]
    description = description_element.childNodes[0].nodeValue
    world_width = float(parse_style(description)['width'])
    scale = world_width / width
    world_height = height * scale
    transform = (euclid.Matrix3.new_translate(*level.start) *
                 euclid.Matrix3.new_scale(scale, -scale) *
                 euclid.Matrix3.new_translate(0, -height))
    for child_node in svg_element.childNodes:
        if child_node.nodeType == minidom.Node.ELEMENT_NODE:
            parse_element(child_node, transform, level)

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

def parse_element(element, transform, level):
    transform_str = element.getAttribute('transform')
    if transform_str:
        transform = transform * parse_transform(transform_str)
    if element.nodeName == 'g':
        for child_node in element.childNodes:
            if child_node.nodeType == minidom.Node.ELEMENT_NODE:
                parse_element(child_node, transform, level)
    elif element.nodeName == 'path':
        if element.getAttribute('sodipodi:type') == 'arc':
            parse_circle_element(element, transform, level)
        else:
            parse_path_element(element, transform, level)
    elif element.nodeName == 'rect':
        parse_rect_element(element, transform, level)
    else:
        log('parse_element(): unsupported SVG element: ' + str(element))

def parse_element_data(element):
    for child_node in element.childNodes:
        if (child_node.nodeType == minidom.Node.ELEMENT_NODE and
            child_node.nodeName == 'desc'):
            return parse_style(child_node.childNodes[0].nodeValue)
    return {}

def parse_path_element(element, transform, level):
    element_data = parse_element_data(element)
    if element_data.get('type') == 'start':
        x = float(element.getAttribute('sodipodi:cx'))
        y = float(element.getAttribute('sodipodi:cy'))
        level.start = transform * euclid.Point2(x, y)
    elif element_data.get('type') == 'goal':
        x = float(element.getAttribute('sodipodi:cx'))
        y = float(element.getAttribute('sodipodi:cy'))
        level.goal = transform * euclid.Point2(x, y)

def parse_circle_element(element, transform, level):
    element_data = parse_element_data(element)
    cx = float(element.getAttribute('sodipodi:cx'))
    cy = float(element.getAttribute('sodipodi:cy'))
    rx = float(element.getAttribute('sodipodi:rx'))
    ry = float(element.getAttribute('sodipodi:ry'))
    position = transform * euclid.Point2(cx, cy)
    shape_def = b2.b2CircleDef()
    shape_def.radius = abs(transform * euclid.Vector2((rx + ry) / 2, 0))
    shape_def.density = float(element_data.get('density', '0'))
    BodyActor(level.world, shape_def, position=position)

def parse_rect_element(element, transform, level):
    element_data = parse_element_data(element)
    x = float(element.getAttribute('x'))
    y = float(element.getAttribute('y'))
    width = float(element.getAttribute('width'))
    height = float(element.getAttribute('height'))
    vertices = [euclid.Point2(x, y),
                euclid.Point2(x, y + height),
                euclid.Point2(x + width, y + height),
                euclid.Point2(x + width, y)]
    vertices = [tuple(transform * v) for v in vertices]
    shape_def = b2.b2PolygonDef()
    shape_def.vertices = vertices
    shape_def.density = float(element_data.get('density', '0'))
    BodyActor(level.world, shape_def)
