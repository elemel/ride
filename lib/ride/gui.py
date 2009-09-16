from __future__ import division

from actors import *
from game import *
import svg

import pyglet
from pyglet.gl import *

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
        self.clock_display = pyglet.clock.ClockDisplay()
        self.time = 0
        self.world_time = 0
        level_model = svg.load_level('lib/ride/levels/basement.svg')
        svg.load_vehicle('lib/ride/vehicles/buggy.svg', level_model)
        self.level_actor = LevelActor(level_model)
        pyglet.clock.schedule_interval(self.step, config.dt)

    def delete(self):
        pyglet.clock.unschedule(self.step)
        super(GameScreen, self).delete()

    def step(self, dt):
        self.time += dt
        while self.world_time + config.dt <= self.time:
            self.world_time += config.dt
            self.level_actor.step(config.dt)

    def on_draw(self):
        self.window.clear()
        glPushMatrix()
        glTranslatef(self.window.width // 2, self.window.height // 2, 0)
        scale = self.window.height / config.camera_height
        glScalef(scale, scale, scale)
        frames = []
        if frames:
            camera_position = frames[0].body.GetWorldCenter()
        else:
            camera_position = self.level_actor.start
        glTranslatef(-camera_position.x, -camera_position.y, 0)
        self.level_actor.debug_draw()
        glPopMatrix()
        if config.fps:
            self.clock_display.draw()
        return pyglet.event.EVENT_HANDLED

    def on_key_press(self, symbol, modifiers):
        if symbol == pyglet.window.key.ESCAPE:
            self.delete()
        if symbol == pyglet.window.key.SPACE:
            self.level_actor.throttle = 1
        if symbol == pyglet.window.key.LEFT:
            self.level_actor.spin += 1
        if symbol == pyglet.window.key.RIGHT:
            self.level_actor.spin -= 1
        return pyglet.event.EVENT_HANDLED

    def on_key_release(self, symbol, modifiers):
        if symbol == pyglet.window.key.SPACE:
            self.level_actor.throttle = 0
        if symbol == pyglet.window.key.LEFT:
            self.level_actor.spin -= 1
        if symbol == pyglet.window.key.RIGHT:
            self.level_actor.spin += 1
        return pyglet.event.EVENT_HANDLED
