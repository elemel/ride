from __future__ import division

import config

from Box2D import *
import euclid
import math
import pyglet
from pyglet.gl import *
import sys
from xml.dom import minidom

def log(message):
    sys.stderr.write('ride: %s\n' % str(message))

def sign(x):
    return -1 if x < 0 else 1

def create_circle_def(radius, density=0, friction=0.5, restitution=0.5,
                      group_index=0):
    shape_def = b2CircleDef()
    shape_def.radius = radius
    shape_def.density = density
    shape_def.friction = friction
    shape_def.restitution = restitution
    shape_def.filter.groupIndex = group_index
    return shape_def

def create_polygon_def(vertices, density=0, friction=0.5, restitution=0.5,
                       group_index=0):
    shape_def = b2PolygonDef()
    shape_def.vertices = vertices
    shape_def.density = density
    shape_def.friction = friction
    shape_def.restitution = restitution
    shape_def.filter.groupIndex = group_index
    return shape_def

def create_box_def(half_width, half_height, density=0, friction=0.5,
                   restitution=0.5, group_index=0):
    shape_def = b2PolygonDef()
    shape_def.SetAsBox(half_width, half_height)
    shape_def.density = density
    shape_def.friction = friction
    shape_def.restitution = restitution
    shape_def.filter.groupIndex = group_index
    return shape_def

class Screen(object):
    def __init__(self, window):
        self.window = window
        window.push_handlers(self)

    def delete(self):
        self.window.pop_handlers()

class TitleScreen(Screen):
    def __init__(self, window):
        super(TitleScreen, self).__init__(window)

    def on_draw(self):
        self.window.clear()
        return pyglet.event.EVENT_HANDLED

    def on_key_press(self, symbol, modifiers):
        if symbol == pyglet.window.key.ESCAPE:
            self.delete()
            self.window.close()
        if symbol == pyglet.window.key.ENTER:
            GameScreen(self.window)
        return pyglet.event.EVENT_HANDLED

class Level(object):
    def __init__(self, lower_bound, upper_bound):
        self.init_world(lower_bound, upper_bound)
        self.start = None
        self.goal = None
        self.vehicle = None
        self.throttle = 0
        self.spin = 0

    def init_world(self, lower_bound, upper_bound):
        aabb = b2AABB()
        aabb.lowerBound = lower_bound
        aabb.upperBound = upper_bound
        self.world = b2World(aabb, config.gravity, True)

    def step(self, dt):
        for spring in self.vehicle.springs:
            spring.step(dt)

        if self.throttle:
            motor_torque = (-config.motor_torque -
                            config.motor_damping *
                            self.vehicle.back_wheel.body.angularVelocity)
            self.vehicle.back_wheel.body.ApplyTorque(motor_torque)

        spin_torque = (self.spin * config.spin_torque -
                       config.spin_damping *
                       self.vehicle.frame.body.angularVelocity)
        self.vehicle.frame.body.ApplyTorque(spin_torque)

        self.world.Step(dt, 10, 10)

    def debug_draw(self):
        for body in self.world.bodyList:
            actor = body.userData
            if actor is not None:
                actor.debug_draw()
        for joint in self.world.jointList:
            if isinstance(joint, b2DistanceJoint):
                glBegin(GL_LINES)
                glVertex2f(*joint.GetAnchor1().tuple())
                glVertex2f(*joint.GetAnchor2().tuple())
                glEnd()
        for spring in self.vehicle.springs:
            glBegin(GL_LINES)
            glVertex2f(*spring.anchor_1.tuple())
            glVertex2f(*spring.anchor_2.tuple())
            glEnd()

def parse_style(style):
    lines = (l.strip() for l in style.split(';'))
    pairs = (l.split(':') for l in lines if l)
    return dict((k.strip(), v.strip()) for k, v in pairs)

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

def parse_rect_element(element, transform, level):
    x = float(element.getAttribute('x'))
    y = float(element.getAttribute('y'))
    width = float(element.getAttribute('width'))
    height = float(element.getAttribute('height'))
    vertices = [euclid.Point2(x, y),
                euclid.Point2(x, y + height),
                euclid.Point2(x + width, y + height),
                euclid.Point2(x + width, y)]
    vertices = [tuple(transform * v) for v in vertices]
    shape_def = b2PolygonDef()
    shape_def.vertices = vertices
    BodyActor(level.world, shape_def)

class GameScreen(Screen):
    def __init__(self, window):
        super(GameScreen, self).__init__(window)
        self.clock_display = pyglet.clock.ClockDisplay()
        self.time = 0
        self.world_time = 0
        self.level = load_level('lib/ride/levels/basement.svg')
        self.level.vehicle = Vehicle(self.level.world, self.level.start)
        pyglet.clock.schedule_interval(self.step, config.dt)

    def delete(self):
        pyglet.clock.unschedule(self.step)
        super(GameScreen, self).delete()

    def step(self, dt):
        self.time += dt
        while self.world_time + config.dt <= self.time:
            self.world_time += config.dt
            self.level.step(config.dt)

    def on_draw(self):
        self.window.clear()
        glPushMatrix()
        glTranslatef(self.window.width // 2, self.window.height // 2, 0)
        scale = self.window.height / config.camera_height
        glScalef(scale, scale, scale)
        camera_position = self.level.vehicle.frame.body.position
        glTranslatef(-camera_position.x, -camera_position.y, 0)
        self.level.debug_draw()
        glPopMatrix()
        if config.fps:
            self.clock_display.draw()
        return pyglet.event.EVENT_HANDLED

    def on_key_press(self, symbol, modifiers):
        if symbol == pyglet.window.key.ESCAPE:
            self.delete()
        if symbol == pyglet.window.key.SPACE:
            self.level.throttle = 1
        if symbol == pyglet.window.key.LEFT:
            self.level.spin += 1
        if symbol == pyglet.window.key.RIGHT:
            self.level.spin -= 1
        return pyglet.event.EVENT_HANDLED

    def on_key_release(self, symbol, modifiers):
        if symbol == pyglet.window.key.SPACE:
            self.level.throttle = 0
        if symbol == pyglet.window.key.LEFT:
            self.level.spin -= 1
        if symbol == pyglet.window.key.RIGHT:
            self.level.spin += 1
        return pyglet.event.EVENT_HANDLED

class Actor(object):
    def __init__(self):
        self.debug_display_list = None

    def delete(self):
        self.delete_debug_display_list()

    def delete_debug_display_list(self):
        if self.debug_display_list is not None:
            glDeleteLists(self.debug_display_list, 1)
            self.debug_display_list = None

    def draw_debug_display_list(self):
        if self.debug_display_list is None:
            self.debug_display_list = glGenLists(1)
            glNewList(self.debug_display_list, GL_COMPILE_AND_EXECUTE)
            self.populate_debug_display_list()
            glEndList()
        else:
            glCallList(self.debug_display_list)

    def populate_debug_display_list(self):
        raise NotImplementedError()

class BodyActor(Actor):
    def __init__(self, world, shape_def, position=(0, 0), angle=0,
                 linear_velocity=(0, 0), angular_velocity=0):
        super(BodyActor, self).__init__()
        self.world = world
        body_def = b2BodyDef()
        body_def.position = position
        body_def.angle = angle
        self.body = self.world.CreateBody(body_def)
        self.body.userData = self
        self.body.CreateShape(shape_def)
        self.body.SetMassFromShapes()
        self.body.linearVelocity = linear_velocity
        self.body.angularVelocity = angular_velocity

    def delete(self):
        self.world.DestroyBody(self.body)
        super(BodyActor, self).delete()

    def debug_draw(self):
        glColor3f(0, 1, 0)
        glPushMatrix()
        glTranslatef(self.body.position.x, self.body.position.y, 0)
        glRotatef(self.body.angle * 180 / math.pi, 0, 0, 1)
        self.draw_debug_display_list()
        glPopMatrix()

    def populate_debug_display_list(self):
        for shape in self.body.shapeList:
            glBegin(GL_LINE_LOOP)
            if isinstance(shape, b2PolygonShape):
                for x, y in shape.vertices:
                    glVertex2f(x, y)
            elif isinstance(shape, b2CircleShape):
                for i in xrange(config.debug_circle_vertex_count):
                    angle = 2 * math.pi * i / config.debug_circle_vertex_count
                    glVertex2f(shape.radius * math.cos(angle),
                               shape.radius * math.sin(angle))
            else:
                assert False
            glEnd(GL_LINE_LOOP)

class Spring(object):
    def __init__(self, body_1, body_2, anchor_1, anchor_2,
                 spring_constant=1, damping=0, max_force=1):
        self.body_1 = body_1
        self.body_2 = body_2
        self._anchor_1 = self.body_1.GetLocalPoint(anchor_1)
        self._anchor_2 = self.body_2.GetLocalPoint(anchor_2)
        self.spring_constant = spring_constant
        self.damping = damping
        self.max_force = max_force
        self.length = (anchor_2 - anchor_1).Length()

    @property
    def anchor_1(self):
        return self.body_1.GetWorldPoint(self._anchor_1)

    @property
    def anchor_2(self):
        return self.body_2.GetWorldPoint(self._anchor_2)

    def step(self, dt):
        direction = self.anchor_2 - self.anchor_1
        length = direction.Normalize()

        # Calculate spring force (without damping).
        spring_force = self.spring_constant * (length - self.length)

        # Calculate damping force.
        relative_velocity = (self.get_linear_velocity_in_point(self.body_2,
                                                               self.anchor_2) -
                             self.get_linear_velocity_in_point(self.body_1,
                                                               self.anchor_1))
        damping_force = self.damping * b2Dot(relative_velocity, direction)

        force = spring_force + damping_force
        force = sign(force) * min(abs(force), self.max_force)
        self.body_1.ApplyForce(force * direction, self.anchor_1)
        self.body_2.ApplyForce(-force * direction, self.anchor_2)

    def get_linear_velocity_in_point(self, body, point):
        offset = point - body.GetWorldCenter()
        return (body.linearVelocity +
                b2Vec2(-offset.y, offset.x) * body.angularVelocity)

class Vehicle(object):
    def __init__(self, world, position):
        self.world = world
        self.group_index = -1
        self.springs = []
        self.init_frame(position)
        self.init_back_wheel(position)
        self.init_front_wheel(position)
        self.init_back_spring()
        self.init_front_spring()

    def delete(self):
        self.frame.delete()
        self.back_wheel.delete()
        self.front_wheel.delete()

    def init_frame(self, position):
        x, y = position
        shape_def = create_box_def(0.75, 0.5, density=1,
                                   group_index=self.group_index)
        self.frame = BodyActor(self.world, shape_def=shape_def,
                               position=(x, y + config.wheel_radius + 0.5))

    def init_back_wheel(self, position):
        x, y = position
        shape_def = create_circle_def(radius=config.wheel_radius,
                                      density=config.wheel_density,
                                      friction=config.wheel_friction,
                                      group_index=self.group_index)
        self.back_wheel = BodyActor(self.world, shape_def=shape_def,
                                    position=(x + config.wheel_distance / -2,
                                              y + config.wheel_radius))

    def init_front_wheel(self, position):
        x, y = position
        shape_def = create_circle_def(radius=config.wheel_radius,
                                      density=config.wheel_density,
                                      friction=config.wheel_friction,
                                      group_index=self.group_index)
        self.front_wheel = BodyActor(self.world, shape_def=shape_def,
                                     position=(x + config.wheel_distance / 2,
                                               y + config.wheel_radius))

    def init_back_spring(self):
        frame_anchor = (self.back_wheel.body.position +
                        b2Vec2(config.wheel_distance / 2, 0))
        joint_def = b2DistanceJointDef()
        joint_def.Initialize(self.frame.body, self.back_wheel.body,
                             frame_anchor, self.back_wheel.body.position)
        self.world.CreateJoint(joint_def)
        self.springs.append(Spring(self.frame.body, self.back_wheel.body,
                                      self.back_wheel.body.position +
                                      b2Vec2(0.5, 1),
                                      self.back_wheel.body.position,
                                      spring_constant=config.back_spring_constant,
                                      damping=config.back_spring_damping,
                                      max_force = config.back_spring_max_force))

    def init_front_spring(self):
        frame_anchor = (self.front_wheel.body.position -
                        b2Vec2(config.wheel_distance / 2, 0))
        joint_def = b2DistanceJointDef()
        joint_def.Initialize(self.frame.body, self.front_wheel.body,
                             frame_anchor, self.front_wheel.body.position)
        self.world.CreateJoint(joint_def)
        self.springs.append(Spring(self.frame.body, self.front_wheel.body,
                                   self.front_wheel.body.position +
                                   b2Vec2(-0.5, 1),
                                   self.front_wheel.body.position,
                                   spring_constant=config.front_spring_constant,
                                   damping=config.front_spring_damping,
                                   max_force=config.front_spring_max_force))

def main():
    window = pyglet.window.Window(fullscreen=config.fullscreen)
    window.set_exclusive_keyboard(window.fullscreen)
    window.set_exclusive_mouse(window.fullscreen)
    TitleScreen(window)
    pyglet.app.run()

if __name__ == '__main__':
    main()
