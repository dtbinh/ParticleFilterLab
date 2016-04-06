import lab10_map
import math
from array import *
import numpy as np


class ParticleFilter:
    def __init__(self):
        self.commandGiven = False
        # self.particles = array()
        self.muTheta = 0
        self.sigma = 0.05
        self.muD = 0.5  #mean
        self.scale = 1000
        initialTuple = (0.5, 0.5, 0.1, 0)
        self.particles = [initialTuple]
        for i in range(0,1,1):
            tuple = (1, 1, 0.1, np.pi/2)
            self.particles.append(tuple)

    def computeParticleD(self, scale, muD, muTheta, sigma):
        # for each particle
        for i in range(0, 2,1):
            d = np.random.normal(muD, scale, sigma)
            theta = np.random.normal(muTheta, scale, sigma)
            # generate x and y positions
            initialX = self.particles[i][0]
            print("initial theta", muTheta)

            initialY = self.particles[i][1]
            initialZ = self.particles[i][2]
            initialTheta = self.particles[i][3]
            value = d * math.cos(initialTheta + theta)
            value2 = d * math.cos(initialTheta + theta)


            self.particles[i][0] = initialX + value #d*np.cos(initialTheta + theta)
            self.particles[i][1] = initialY + value2 #d*np.cos(initialTheta + theta)
            self.particles[i][2] = initialZ
            self.particles[i][3] = theta



    def recieveCommand(self, _muTheta, _muDistance):
        # 90 = turn right, -90 = turn left, 0 = move forward
        self.muTheta = _muTheta
        self.muD = _muDistance
        print("recieved command")
        self.commandGiven = True
        self.computeParticleD(self.scale, self.muD, self.muTheta, self.sigma)