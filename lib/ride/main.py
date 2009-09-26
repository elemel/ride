from __future__ import division

import settings
from gui import *

import pyglet

def main():
    window = pyglet.window.Window(fullscreen=settings.fullscreen)
    window.set_exclusive_keyboard(window.fullscreen)
    window.set_exclusive_mouse(window.fullscreen)
    TitleScreen(window)
    pyglet.app.run()

if __name__ == '__main__':
    main()
