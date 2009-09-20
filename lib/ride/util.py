from __future__ import division

import math
from pyglet.gl import *
import sys

def sign(x):
    return -1 if x < 0 else 1

def log(message):
    sys.stderr.write('ride: %s\n' % str(message))

class CircleDisplayList(object):
    def __init__(self, vertex_count=100):
        assert vertex_count >= 1
        self.display_list = glGenLists(1)
        assert self.display_list
        glNewList(self.display_list, GL_COMPILE)
        glBegin(GL_LINE_LOOP)
        for i in xrange(vertex_count):
            angle = 2 * math.pi * i / vertex_count
            glVertex2f(math.cos(angle), math.sin(angle))
        glEnd()
        glEndList()

    def delete(self):
        if self.display_list:
            glDeleteLists(self.display_list, 1)
            self.display_list = 0

    def draw(self, center=(0, 0), radius=1):
        assert self.display_list
        x, y = center
        if x == 0 and y == 0 and radius == 1:
            glCallList(self.display_list)
        else:
            glPushMatrix()
            glTranslatef(x, y, 0)
            glScalef(radius, radius, 1)
            glCallList(self.display_list)
            glPopMatrix()

def save_screenshot(name='screenshot.png', format='RGB'):
    image = pyglet.image.get_buffer_manager().get_color_buffer().image_data
    image.format = format
    image.save(name)
