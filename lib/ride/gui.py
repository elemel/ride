from __future__ import division

from actors import *
import settings
import svg
from utils import *

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
                                       anchor_x='center', anchor_y='center',
                                       color=(0xee, 0, 0x11, 255))

    def on_draw(self):
        glClearColor(1, 1, 1, 1)
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
        level_model = svg.load_level('lib/ride/levels/bumps.svg')
        svg.load_vehicle('lib/ride/vehicles/buggy.svg', level_model)
        self.level_actor = LevelActor(level_model)
        pyglet.clock.schedule_interval(self.step, settings.dt)

    def delete(self):
        pyglet.clock.unschedule(self.step)
        super(GameScreen, self).delete()

    def step(self, dt):
        self.time += dt
        while self.world_time + settings.dt <= self.time:
            self.world_time += settings.dt
            self.level_actor.step(settings.dt)

    def on_draw(self):
        glClearColor(*self.level_actor.background_color)
        self.window.clear()
        glColor4f(*self.level_actor.color)
        glPushMatrix()
        glTranslatef(self.window.width // 2, self.window.height // 2, 0)
        scale = self.window.height / settings.camera_height
        glScalef(scale, scale, scale)
        camera_x, camera_y = self.level_actor.camera
        glTranslatef(-camera_x, -camera_y, 0)
        if settings.debug:
            self.level_actor.debug_draw()
        else:
            self.level_actor.draw()
        glPopMatrix()
        if settings.fps:
            self.clock_display.draw()
        return pyglet.event.EVENT_HANDLED

    def on_key_press(self, symbol, modifiers):
        if symbol == pyglet.window.key.ESCAPE:
            self.delete()
        elif symbol == pyglet.window.key.F12:
            save_screenshot('ride-screenshot.png')
        else:
            self.level_actor.on_key_press(symbol, modifiers)
        return pyglet.event.EVENT_HANDLED

    def on_key_release(self, symbol, modifiers):
        self.level_actor.on_key_release(symbol, modifiers)
        return pyglet.event.EVENT_HANDLED
