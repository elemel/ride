from __future__ import division

import b2
from models import *
from util import *

import math
from pyglet.gl import *

class Actor(object):
    pass

class LevelActor(Actor):
    def __init__(self, level_model):
        aabb = b2.b2AABB()
        aabb.lowerBound = level_model.lower_bound
        aabb.upperBound = level_model.upper_bound
        self.world = b2.b2World(aabb, level_model.gravity, True)
        self.start = level_model.start
        self.goal = level_model.goal
        self.extra_joint_actors = []

        for body_model in level_model.body_models:
            BodyActor(self, body_model)
        for joint_model in level_model.joint_models:
            self.create_joint(joint_model)

        self.circle_display_list = CircleDisplayList()

    def get_bodies_at_point(self, point):
        segment = b2.b2Segment()
        segment.p1 = tuple(point)
        segment.p2 = tuple(point)
        count, shapes = self.world.Raycast(segment, 1000, True, None)
        return list(set(s.GetBody() for s in shapes))

    def get_bodies_at_line_segment(self, p1, p2):
        bodies_1 = self.get_bodies_at_point(p1)
        bodies_2 = self.get_bodies_at_point(p2)
        if len(bodies_1) == 1 and len(bodies_2) == 2:
            bodies_2.remove(bodies_1[0])
        if len(bodies_1) == 2 and len(bodies_2) == 1:
            bodies_1.remove(bodies_2[0])
        return bodies_1[0], bodies_2[0]

    def create_joint(self, joint_model):
        if isinstance(joint_model, DistanceJointModel):
            bodies = self.get_bodies_at_line_segment(joint_model.anchor_1,
                                                     joint_model.anchor_2)
            joint_def = b2.b2DistanceJointDef()
            joint_def.Initialize(bodies[0], bodies[1],
                                 tuple(joint_model.anchor_1),
                                 tuple(joint_model.anchor_2))
            self.world.CreateJoint(joint_def)
        elif isinstance(joint_model, SpringModel):
            SpringActor(self, joint_model)
        else:
            assert False

    def step(self, dt):
        """
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
        """

        for joint_actor in self.extra_joint_actors:
            joint_actor.step(dt)
        self.world.Step(dt, 10, 10)

    def debug_draw(self):
        for body in self.world.bodyList:
            self.debug_draw_body(body)
        for joint in self.world.jointList:
            if isinstance(joint, b2.b2DistanceJoint):
                glBegin(GL_LINES)
                glVertex2f(*joint.GetAnchor1().tuple())
                glVertex2f(*joint.GetAnchor2().tuple())
                glEnd()
        """
        for spring in self.springs:
            glBegin(GL_LINES)
            glVertex2f(*spring.anchor_1.tuple())
            glVertex2f(*spring.anchor_2.tuple())
            glEnd()
        """

    def debug_draw_body(self, body):
        glPushMatrix()
        x, y = body.position.tuple()
        glTranslatef(x, y, 0)
        glRotatef(body.angle * 180 / math.pi, 0, 0, 1)
        for shape in body.shapeList:
            if isinstance(shape, b2.b2PolygonShape):
                glBegin(GL_LINE_LOOP)
                for x, y in shape.vertices:
                    glVertex2f(x, y)
                glEnd()
            elif isinstance(shape, b2.b2CircleShape):
                self.circle_display_list.draw(shape.localPosition.tuple(),
                                              shape.radius)
        glPopMatrix()

class BodyActor(Actor):
    def __init__(self, level_actor, body_model):
        body_def = b2.b2BodyDef()
        self.body = level_actor.world.CreateBody(body_def)
        self.body.userData = self
        for shape_model in body_model.shape_models:
            if isinstance(shape_model, CircleModel):
                shape_def = b2.b2CircleDef()
                shape_def.localPosition = self.body.GetLocalPoint(tuple(shape_model.center))
                shape_def.radius = shape_model.radius
            elif isinstance(shape_model, PolygonModel):
                shape_def = b2.b2PolygonDef()
                shape_def.vertices = [self.body.GetLocalPoint(tuple(v))
                                      for v in shape_model.vertices]
            else:
                assert False
            shape_def.density = shape_model.density
            shape_def.friction = shape_model.friction
            shape_def.restitution = shape_model.restitution
            self.body.CreateShape(shape_def)
        self.body.SetMassFromShapes()

class SpringActor(object):
    def __init__(self, level_actor, spring_model):
        anchor_1 = tuple(spring_model.anchor_1)
        anchor_2 = tuple(spring_model.anchor_2)
        bodies = level_actor.get_bodies_at_line_segment(anchor_1, anchor_2)
        self.body_1, self.body_2 = bodies
        self._anchor_1 = self.body_1.GetLocalPoint(anchor_1)
        self._anchor_2 = self.body_2.GetLocalPoint(anchor_2)
        self.spring_constant = spring_model.spring_constant
        self.damping_constant = spring_model.damping_constant
        self.max_force = 1000 # spring_model.max_force
        self.length = abs(spring_model.anchor_2 - spring_model.anchor_1)
        level_actor.extra_joint_actors.append(self)

    @property
    def anchor_1(self):
        return self.body_1.GetWorldPoint(self._anchor_1)

    @property
    def anchor_2(self):
        return self.body_2.GetWorldPoint(self._anchor_2)

    def step(self, dt):
        direction = self.anchor_2 - self.anchor_1
        length = direction.Normalize()

        # Calculate spring force.
        spring_force = self.spring_constant * (length - self.length)

        # Calculate damping force.
        relative_velocity = (self.get_linear_velocity_in_point(self.body_2,
                                                               self.anchor_2) -
                             self.get_linear_velocity_in_point(self.body_1,
                                                               self.anchor_1))
        damping_force = self.damping_constant * b2.b2Dot(relative_velocity,
                                                         direction)

        force = spring_force + damping_force
        force = sign(force) * min(abs(force), self.max_force)
        self.body_1.ApplyForce(force * direction, self.anchor_1)
        self.body_2.ApplyForce(-force * direction, self.anchor_2)

    def get_linear_velocity_in_point(self, body, point):
        offset = point - body.GetWorldCenter()
        return (body.linearVelocity +
                b2.b2Vec2(-offset.y, offset.x) * body.angularVelocity)
