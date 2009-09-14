from __future__ import division

import b2
import config
from util import *

from collections import defaultdict
import math
from pyglet.gl import *

class Level(object):
    def __init__(self, lower_bound, upper_bound, gravity):
        self.init_world(lower_bound, upper_bound, gravity)
        self.start = None
        self.goal = None
        self.vehicle = None
        self.throttle = 0
        self.spin = 0
        self.springs = []
        self.labels = defaultdict(list)

    def init_world(self, lower_bound, upper_bound, gravity):
        aabb = b2.b2AABB()
        aabb.lowerBound = lower_bound
        aabb.upperBound = upper_bound
        self.world = b2.b2World(aabb, gravity, True)

    def step(self, dt):
        if self.throttle:
            back_wheel = self.labels['back-wheel'][0]
            motor_torque = (-back_wheel.motor_torque -
                            back_wheel.motor_damping *
                            back_wheel.body.angularVelocity)
            back_wheel.body.ApplyTorque(motor_torque)

        if self.spin:
            frame = self.labels['frame'][0]
            spin_torque = (self.spin * frame.motor_torque -
                           frame.motor_damping * frame.body.angularVelocity)
            frame.body.ApplyTorque(spin_torque)

        for spring in self.springs:
            spring.step(dt)
        self.world.Step(dt, 10, 10)

    def debug_draw(self):
        for body in self.world.bodyList:
            actor = body.userData
            if actor is not None:
                actor.debug_draw()
        for joint in self.world.jointList:
            if isinstance(joint, b2.b2DistanceJoint):
                glBegin(GL_LINES)
                glVertex2f(*joint.GetAnchor1().tuple())
                glVertex2f(*joint.GetAnchor2().tuple())
                glEnd()
        for spring in self.springs:
            glBegin(GL_LINES)
            glVertex2f(*spring.anchor_1.tuple())
            glVertex2f(*spring.anchor_2.tuple())
            glEnd()

    def add_spring(self, spring):
        self.springs.append(spring)

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
    def __init__(self, level, shape_def, position=(0, 0), angle=0,
                 linear_velocity=(0, 0), angular_velocity=0, label=None,
                 motor_torque=0, motor_damping=0):
        super(BodyActor, self).__init__()
        self.level = level
        self.level.labels[label].append(self)
        body_def = b2.b2BodyDef()
        body_def.position = tuple(position)
        body_def.angle = angle
        self.body = self.level.world.CreateBody(body_def)
        self.body.userData = self
        self.body.CreateShape(shape_def)
        self.body.SetMassFromShapes()
        self.body.linearVelocity = linear_velocity
        self.body.angularVelocity = angular_velocity
        self.motor_torque = motor_torque
        self.motor_damping = motor_damping

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
            if isinstance(shape, b2.b2PolygonShape):
                for x, y in shape.vertices:
                    glVertex2f(x, y)
            elif isinstance(shape, b2.b2CircleShape):
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
        damping_force = self.damping * b2.b2Dot(relative_velocity, direction)

        force = spring_force + damping_force
        force = sign(force) * min(abs(force), self.max_force)
        self.body_1.ApplyForce(force * direction, self.anchor_1)
        self.body_2.ApplyForce(-force * direction, self.anchor_2)

    def get_linear_velocity_in_point(self, body, point):
        offset = point - body.GetWorldCenter()
        return (body.linearVelocity +
                b2.b2Vec2(-offset.y, offset.x) * body.angularVelocity)

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
        shape_def = b2.b2PolygonDef()
        shape_def.SetAsBox(0.75, 0.5)
        shape_def.density = 1
        shape_def.filter.groupIndex = self.group_index
        self.frame = BodyActor(self.world, shape_def=shape_def,
                               position=(x, y + config.wheel_radius + 0.5))

    def init_back_wheel(self, position):
        x, y = position
        shape_def = b2.b2CircleDef()
        shape_def.radius = config.wheel_radius
        shape_def.density= config.wheel_density
        shape_def.friction = config.wheel_friction
        shape_def.filter.groupIndex = self.group_index
        self.back_wheel = BodyActor(self.world, shape_def=shape_def,
                                    position=(x + config.wheel_distance / -2,
                                              y + config.wheel_radius))

    def init_front_wheel(self, position):
        x, y = position
        shape_def = b2.b2CircleDef()
        shape_def.radius = config.wheel_radius
        shape_def.density= config.wheel_density
        shape_def.friction = config.wheel_friction
        shape_def.filter.groupIndex = self.group_index
        self.front_wheel = BodyActor(self.world, shape_def=shape_def,
                                     position=(x + config.wheel_distance / 2,
                                               y + config.wheel_radius))

    def init_back_spring(self):
        frame_anchor = (self.back_wheel.body.position +
                        b2.b2Vec2(config.wheel_distance / 2, 0))
        joint_def = b2.b2DistanceJointDef()
        joint_def.Initialize(self.frame.body, self.back_wheel.body,
                             frame_anchor, self.back_wheel.body.position)
        self.world.CreateJoint(joint_def)
        self.springs.append(Spring(self.frame.body, self.back_wheel.body,
                                   self.back_wheel.body.position +
                                   b2.b2Vec2(0.5, 1),
                                   self.back_wheel.body.position,
                                   spring_constant=config.back_spring_constant,
                                   damping=config.back_spring_damping,
                                   max_force = config.back_spring_max_force))

    def init_front_spring(self):
        frame_anchor = (self.front_wheel.body.position -
                        b2.b2Vec2(config.wheel_distance / 2, 0))
        joint_def = b2.b2DistanceJointDef()
        joint_def.Initialize(self.frame.body, self.front_wheel.body,
                             frame_anchor, self.front_wheel.body.position)
        self.world.CreateJoint(joint_def)
        self.springs.append(Spring(self.frame.body, self.front_wheel.body,
                                   self.front_wheel.body.position +
                                   b2.b2Vec2(-0.5, 1),
                                   self.front_wheel.body.position,
                                   spring_constant=config.front_spring_constant,
                                   damping=config.front_spring_damping,
                                   max_force=config.front_spring_max_force))
