from math import *
from random import gauss
import random


landmarks = [[20.0, 80.0],[80.0, 80.0],[20.0, 80.0],[80.0, 20.0]]
world_size = 100.0

   
class Particle:
    def __init__(self):
        self.x = random.random() * world_size
        self.y = random.random() * world_size
        self.orientation = random.random() * 2.0 * pi
        self.forward_noise = 0.0
        self.turn_noise = 0.0
        self.sense_noise = 0.0

    def set(self, new_x, new_y, new_orientation):
        if new_x < 0 or new_x >= world_size:
            raise ValueError('X coordinate out of bound')
        if new_y < 0 or new_y >= world_size:
            raise ValueError('Y coordinate out of bound')
        if new_orientation < 0 or new_orientation >= world_size:
            raise ValueError('Orientation out of bound')
        self.x = new_x
        self.y = new_y
        self.orientation = new_orientation

    def set_noise(self, new_forward_noise, new_turn_noise, new_sense_noise):
        self.forward_noise = new_forward_noise
        self.turn_noise = new_turn_noise
        self.sense_noise = new_sense_noise

    def sense(self):
        z = []
        for i in range (len(landmarks)):
            dist = sqrt((self.x - landmarks[i][0])**2 + (self.y - landmarks[i][1])**2)
            z.append(dist)
        return z

    def move(self, turn, forward):
        if forward < 0:
            raise ValueError('Robot cannot move backwards')
        orientation = self.orientation + float(turn) + gauss(0.0, self.turn_noise)
        orientation %= 2 * pi

        dist = float(forward) + random.gauss(0.0, self.forward_noise)
        x = self.x + (cos(orientation) * dist)
        y = self.y + (sin(orientation) * dist)
        x %= world_size
        y %= world_size

        res = Particle()
        res.set(x, y, orientation)
        res.set_noise(self.forward_noise, self.turn_noise, self.sense_noise)
        return res

    def Gaussian(self, mu, sigma, curr_x):
        return exp(- ((mu - curr_x) ** 2) / (sigma ** 2) / 2.0) / sqrt(2.0 * pi * (sigma ** 2))

    def measurement_prob(self, measurement):
        prob = 1.0
        for i in range(len(landmarks)):
            dist = sqrt((self.x - landmarks[i][0])**2 + (self.y - landmarks[i][1])**2)
            prob *= self.Gaussian(dist, self.sense_noise, measurement[i])
        return prob

    def __repr__(self):
        return '[x=%.6s  y=%.6s  orient=%.6s]' %(str(self.x), str(self.y), str(self.orientation))


##########################################################################################################

'''
    Patricle filter implementation
'''
my_robot = Particle()
my_robot = my_robot.move(0.1, 5.0)
Z = my_robot.sense()
N = 1000
particles = []
for i in range(N):
    particle = Particle()
    particle.set_noise(0.05, 0.05, 5.0)
    particles.append(particle)

particles2 = []
for p in range(N):
    particles2.append(particles[i].move(0.1, 5))

w = []
for i in range (N):
    w.append(particles2[i].measurement_prob(Z))
print(w)