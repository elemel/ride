from __future__ import division

import b2
from models import *
from util import *

import math
from pyglet.gl import *

class Actor(object):
    def __init__(self, model):
        self.id = model.id

    def delete(self):
        pass

class LevelActor(Actor):
    def __init__(self, level_model):
        super(LevelActor, self).__init__(level_model)
        self.color = 0, 0, 0, 1
        self.background_color = 1, 1, 1, 1
        aabb = b2.b2AABB()
        aabb.lowerBound = level_model.lower_bound
        aabb.upperBound = level_model.upper_bound
        self.world = b2.b2World(aabb, level_model.gravity, True)
        self.start = level_model.start
        self.goal = level_model.goal
        self.extra_joint_actors = []
        self.key_names = {
            pyglet.window.key.LEFT: 'left',
            pyglet.window.key.RIGHT: 'right',
            pyglet.window.key.SPACE: 'space',
        }
        self.key_press_bindings = {}
        self.key_release_bindings = {}
        self.z = 1

        for body_model in level_model.body_models:
            BodyActor(self, body_model)
        for joint_model in level_model.joint_models:
            self.create_joint(joint_model)

        self.circle_line_loop_display_list = CircleDisplayList()
        self.circle_polygon_display_list = CircleDisplayList(mode=GL_POLYGON)
        self.display_lists = {}
        self.camera = 0, 0

    def get_bodies_at_point(self, point):
        segment = b2.b2Segment()
        segment.p1 = tuple(point)
        segment.p2 = tuple(point)
        count, shapes = self.world.Raycast(segment, 1000, True, None)
        return list(set(s.GetBody() for s in shapes))

    def get_top_bodies_at_point(self, point, body_count=2):
        def key(body):
            return body.userData.z
        return sorted(self.get_bodies_at_point(point), key=key)[-body_count:]

    def get_top_body_at_point(self, point):
        def key(body):
            return body.userData.z
        return max(self.get_bodies_at_point(point), key=key)

    def create_joint(self, joint_model):
        if isinstance(joint_model, RevoluteJointModel):
            bodies = self.get_top_bodies_at_point(joint_model.anchor)
            if len(bodies) == 1:
                body_1 = bodies[0]
                body_2 = self.world.GetGroundBody()
            else:
                body_1, body_2 = bodies
            joint_def = b2.b2RevoluteJointDef()
            joint_def.Initialize(body_1, body_2, tuple(joint_model.anchor))
            self.world.CreateJoint(joint_def)
        elif isinstance(joint_model, DistanceJointModel):
            body_1 = self.get_top_body_at_point(joint_model.anchor_1)
            body_2 = self.get_top_body_at_point(joint_model.anchor_2)
            joint_def = b2.b2DistanceJointDef()
            joint_def.Initialize(body_1, body_2, tuple(joint_model.anchor_1),
                                 tuple(joint_model.anchor_2))
            self.world.CreateJoint(joint_def)
        elif isinstance(joint_model, PrismaticJointModel):
            body_1 = self.get_top_body_at_point(joint_model.anchor_1)
            body_2 = self.get_top_body_at_point(joint_model.anchor_2)
            joint_def = b2.b2PrismaticJointDef()
            axis = tuple(joint_model.anchor_2 - joint_model.anchor_1)
            joint_def.Initialize(body_1, body_2, tuple(joint_model.anchor_1),
                                 axis)
            self.world.CreateJoint(joint_def)
        elif isinstance(joint_model, SpringModel):
            SpringActor(self, joint_model)
        elif isinstance(joint_model, MotorModel):
            MotorActor(self, joint_model)
        elif isinstance(joint_model, CameraModel):
            CameraActor(self, joint_model)
        else:
            assert False

    def step(self, dt):
        for joint_actor in self.extra_joint_actors:
            joint_actor.step(dt)
        self.world.Step(dt, 10, 10)

    def draw(self):
        for body in self.world.bodyList:
            glPushMatrix()
            x, y = body.position.tuple()
            glTranslatef(x, y, 0)
            glRotatef(body.angle * 180 / math.pi, 0, 0, 1)
            if body not in self.display_lists:
                self.display_lists[body] = glGenLists(1)
                glNewList(self.display_lists[body], GL_COMPILE)
                self.draw_body(body)
                glEndList()
            glCallList(self.display_lists[body])
            glPopMatrix()

    def draw_body(self, body):
        for shape in body.shapeList:
            color = shape.userData
            if color is None:
                color = self.color
            glColor4f(*color)
            if isinstance(shape, b2.b2PolygonShape):
                glBegin(GL_POLYGON)
                for x, y in shape.vertices:
                    glVertex2f(x, y)
                glEnd()
            elif isinstance(shape, b2.b2CircleShape):
                self.circle_polygon_display_list.draw(shape.localPosition.tuple(),
                                                      shape.radius)

    def debug_draw(self):
        for body in self.world.bodyList:
            self.debug_draw_body(body)
        for joint in self.world.jointList:
            if isinstance(joint, b2.b2DistanceJoint):
                glBegin(GL_LINES)
                glVertex2f(*joint.GetAnchor1().tuple())
                glVertex2f(*joint.GetAnchor2().tuple())
                glEnd()
        for joint_actor in self.extra_joint_actors:
            if isinstance(joint_actor, SpringActor):
                glBegin(GL_LINES)
                glVertex2f(*joint_actor.anchor_1.tuple())
                glVertex2f(*joint_actor.anchor_2.tuple())
                glEnd()

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
                self.circle_line_loop_display_list.draw(shape.localPosition.tuple(),
                                                        shape.radius)
        glPopMatrix()

    def on_key_press(self, symbol, modifiers):
        name = self.key_names.get(symbol)
        if name is not None:
            func = self.key_press_bindings.get(name)
            if func is not None:
                func()

    def on_key_release(self, symbol, modifiers):
        name = self.key_names.get(symbol)
        if name is not None:
            func = self.key_release_bindings.get(name)
            if func is not None:
                func()

class BodyActor(Actor):
    def __init__(self, level_actor, body_model):
        super(BodyActor, self).__init__(body_model)
        self.z = level_actor.z
        level_actor.z += 1
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
            shape_def.filter.groupIndex = shape_model.group_index
            shape = self.body.CreateShape(shape_def)
            shape.userData = shape_model.color
        self.body.SetMassFromShapes()

class JointActor(Actor):
    pass

class SpringActor(JointActor):
    def __init__(self, level_actor, spring_model):
        super(SpringActor, self).__init__(spring_model)
        anchor_1 = tuple(spring_model.anchor_1)
        anchor_2 = tuple(spring_model.anchor_2)
        self.body_1 = level_actor.get_top_body_at_point(anchor_1)
        self.body_2 = level_actor.get_top_body_at_point(anchor_2)
        self._anchor_1 = self.body_1.GetLocalPoint(anchor_1)
        self._anchor_2 = self.body_2.GetLocalPoint(anchor_2)
        self.spring_constant = spring_model.spring_constant
        self.damping = spring_model.damping
        self.max_force = 1000 # spring_model.max_force
        self.length = abs(spring_model.anchor_2 - spring_model.anchor_1)
        level_actor.extra_joint_actors.append(self)

    def delete(self):
        self.level_actor.extra_joint_actors.remove(self)

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
        damping_force = self.damping * b2.b2Dot(relative_velocity, direction)

        force = spring_force + damping_force
        force = sign(force) * min(abs(force), self.max_force)
        self.body_1.ApplyForce(force * direction, self.anchor_1)
        self.body_2.ApplyForce(-force * direction, self.anchor_2)

    def get_linear_velocity_in_point(self, body, point):
        offset = point - body.GetWorldCenter()
        return (body.linearVelocity +
                b2.b2Vec2(-offset.y, offset.x) * body.angularVelocity)

class MotorActor(JointActor):
    def __init__(self, level_actor, motor_model):
        super(MotorActor, self).__init__(motor_model)
        self.level_actor = level_actor
        self.body = level_actor.get_top_body_at_point(tuple(motor_model.anchor))
        self.torque = motor_model.torque
        self.damping = motor_model.damping
        self.clockwise_key = motor_model.clockwise_key
        self.counter_clockwise_key = motor_model.counter_clockwise_key
        self.throttle = 0
        self.level_actor.key_press_bindings[self.clockwise_key] = self.decrement_throttle
        self.level_actor.key_release_bindings[self.clockwise_key] = self.increment_throttle
        self.level_actor.key_press_bindings[self.counter_clockwise_key] = self.increment_throttle
        self.level_actor.key_release_bindings[self.counter_clockwise_key] = self.decrement_throttle
        self.level_actor.extra_joint_actors.append(self)

    def delete(self):
        self.level_actor.key_press_bindings[self.clockwise_key] = None
        self.level_actor.key_release_bindings[self.clockwise_key] = None
        self.level_actor.key_press_bindings[self.counter_clockwise_key] = None
        self.level_actor.key_release_bindings[self.counter_clockwise_key] = None
        self.level_actor.extra_joint_actors.remove(self)

    def increment_throttle(self):
        self.throttle += 1

    def decrement_throttle(self):
        self.throttle -= 1

    def step(self, dt):
        if self.throttle:
            torque = (self.throttle * self.torque -
                      self.damping * self.body.angularVelocity)
            self.body.ApplyTorque(torque)

class CameraActor(JointActor):
    def __init__(self, level_actor, camera_model):
        super(CameraActor, self).__init__(camera_model)
        self.level_actor = level_actor
        self.body = level_actor.get_top_body_at_point(tuple(camera_model.anchor))
        self.level_actor.extra_joint_actors.append(self)

    def delete(self):
        self.level_actor.extra_joint_actors.remove(self)

    def step(self, dt):
        self.level_actor.camera = self.body.GetWorldCenter().tuple()
