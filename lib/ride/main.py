from __future__ import division

import config

from Box2D import *
import math
import pyglet
from pyglet.gl import *

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

class GameScreen(Screen):
    def __init__(self, window):
        super(GameScreen, self).__init__(window)
        self.init_world()
        self.init_level()
        self.vehicle = Vehicle(self.world, (0, 0))
        self.clock_display = pyglet.clock.ClockDisplay()
        self.throttle = 0
        self.spin = 0
        self.time = 0
        self.world_time = 0
        pyglet.clock.schedule_interval(self.step, config.dt)

    def init_world(self):
        aabb = b2AABB()
        aabb.lowerBound = -100, -100
        aabb.upperBound = 100, 100
        self.world = b2World(aabb, config.gravity, True)

    def init_level(self):
        BodyActor(self.world, shape_def=create_box_def(5, 0.1),
                  position=(0, -2), angle=-0.3)
        BodyActor(self.world, shape_def=create_box_def(5, 0.1),
                  position=(7, -3))
        BodyActor(self.world, shape_def=create_box_def(5, 0.1),
                  position=(14, -1), angle=0.5)

        BodyActor(self.world, shape_def=create_box_def(5, 0.1),
                  position=(32, 2), angle=0.1)
        BodyActor(self.world, shape_def=create_box_def(10, 0.1),
                  position=(36, -6), angle=-0.2)
        BodyActor(self.world, shape_def=create_box_def(5, 0.1),
                  position=(21, -7), angle=-1)
        BodyActor(self.world, shape_def=create_box_def(5, 0.1),
                  position=(26, -12), angle=-0.5)

    def delete(self):
        pyglet.clock.unschedule(self.step)
        super(GameScreen, self).delete()

    def step(self, dt):
        self.time += dt
        while self.world_time + config.dt <= self.time:
            self.world_time += config.dt

            for spring in self.vehicle.springs:
                spring.step(config.dt)

            if self.throttle:
                motor_torque = (-config.motor_torque -
                                config.motor_damping *
                                self.vehicle.back_wheel.body.angularVelocity)
                self.vehicle.back_wheel.body.ApplyTorque(motor_torque)
                self.vehicle.front_wheel.body.ApplyTorque(motor_torque)

            spin_torque = (self.spin * config.spin_torque -
                           config.spin_damping *
                           self.vehicle.frame.body.angularVelocity)
            self.vehicle.frame.body.ApplyTorque(spin_torque)

            self.world.Step(config.dt, 10, 10)

    def on_draw(self):
        self.window.clear()
        glPushMatrix()
        glTranslatef(self.window.width // 2, self.window.height // 2, 0)
        scale = self.window.height / config.camera_height
        glScalef(scale, scale, scale)
        camera_position = self.vehicle.frame.body.position
        glTranslatef(-camera_position.x, -camera_position.y, 0)
        self.debug_draw()
        glPopMatrix()
        if config.fps:
            self.clock_display.draw()
        return pyglet.event.EVENT_HANDLED

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

    def on_key_press(self, symbol, modifiers):
        if symbol == pyglet.window.key.ESCAPE:
            self.delete()
        if symbol == pyglet.window.key.SPACE:
            self.throttle = 1
        if symbol == pyglet.window.key.LEFT:
            self.spin += 1
        if symbol == pyglet.window.key.RIGHT:
            self.spin -= 1
        return pyglet.event.EVENT_HANDLED

    def on_key_release(self, symbol, modifiers):
        if symbol == pyglet.window.key.SPACE:
            self.throttle = 0
        if symbol == pyglet.window.key.LEFT:
            self.spin -= 1
        if symbol == pyglet.window.key.RIGHT:
            self.spin += 1
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
        self.init_frame()
        self.init_back_wheel()
        self.init_front_wheel()
        self.init_back_spring()
        self.init_front_spring()

    def delete(self):
        self.frame.delete()
        self.back_wheel.delete()
        self.front_wheel.delete()

    def init_frame(self):
        shape_def = create_box_def(0.75, 0.5, density=1,
                                   group_index=self.group_index)
        self.frame = BodyActor(self.world, shape_def=shape_def,
                               position=(0, config.wheel_radius + 0.5))

    def init_back_wheel(self):
        shape_def = create_circle_def(radius=config.wheel_radius,
                                      density=config.wheel_density,
                                      friction=config.wheel_friction,
                                      group_index=self.group_index)
        self.back_wheel = BodyActor(self.world, shape_def=shape_def,
                                    position=(config.wheel_distance / -2,
                                              config.wheel_radius))

    def init_front_wheel(self):
        shape_def = create_circle_def(radius=config.wheel_radius,
                                      density=config.wheel_density,
                                      friction=config.wheel_friction,
                                      group_index=self.group_index)
        self.front_wheel = BodyActor(self.world, shape_def=shape_def,
                                     position=(config.wheel_distance / 2,
                                               config.wheel_radius))

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
