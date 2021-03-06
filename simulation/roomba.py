from random import random, uniform
from vector import *
from math import pi
import time

class Roomba:

    def __init__(self, pos, vel, rCircle):
        self.pos = pos
        self.vel = vel

        self.tick = time.time()
        self.size = 9
        self.randang = time.time()
        self.spike = 0
        self.circle = rCircle
        self.d = 0

    def death(self):
        if self.pos.y <= 25 or self.pos.y >= 625 or self.pos.x <= 25 or self.pos.x >= 625:
            self.d = 1
            print "DEAD"
		#print self.pos.x, self.pos.y

    def step(self):
        self.death()
        current = time.time()
        if current-self.randang > 5:   # time elapsed since last random angle change > 5 sec
            noise = uniform(-pi/9, pi/9)   # maximum noise is 20 degrees  
            self.vel.update_angle(noise)
            self.randang = current

        if current-self.tick > 20: # time elapsed since last directional change > 20 sec
            #print "about to reverse"
            self.vel.update_angle(pi)
            self.tick = current
        else:
            self.pos.add(self.vel)

        #print self.tick
        
        #self.tick = self.tick - 1
        #self.randang = self.randang - 1

