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
        self.label = pyglet.text.Label('RIDE', font_size=200, italic=True,
                                       anchor_x='center', anchor_y='center')

    def on_draw(self):
        self.window.clear()
        self.label.x = self.window.width // 2
        self.label.y = self.window.height // 2
        self.label.draw()
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
        camera_x, camera_y = self.level_actor.camera
        glTranslatef(-camera_x, -camera_y, 0)
        self.level_actor.debug_draw()
        glPopMatrix()
        if config.fps:
            self.clock_display.draw()
        return pyglet.event.EVENT_HANDLED

    def on_key_press(self, symbol, modifiers):
        if symbol == pyglet.window.key.ESCAPE:
            self.delete()
        else:
            self.level_actor.on_key_press(symbol, modifiers)
        return pyglet.event.EVENT_HANDLED

    def on_key_release(self, symbol, modifiers):
        self.level_actor.on_key_release(symbol, modifiers)
        return pyglet.event.EVENT_HANDLED
