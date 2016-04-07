import lab10_map
import math
import random
from array import *
import numpy as np


class ParticleFilter:
    def __init__(self):
        self.commandGiven = False
        # self.particles = array()
        self.muTheta = 0
        self.sigma = 0.05
        self.muD = 0.5  #mean
        self.scale = 100

        self.randomNumbers = random.sample(range(300), 200)
        self.randomTheta = random.sample(range(360), 100)
        initialTuple = (self.randomNumbers[0]/self.scale, self.randomNumbers[1]/self.scale, self.randomNumbers[2]/self.scale, self.randomTheta[0])

        print(self.randomNumbers[0]/100)
        self.particles = [initialTuple]

        for i in range(0,1,1):
            tuple = (self.randomNumbers[3]/self.scale, self.randomNumbers[4]/self.scale, self.randomNumbers[5]/self.scale, self.randomTheta[1])
            self.particles.append(tuple)

    # x = fake sensor reading, mu = actual sensor reading
    def findPDF(self, x ,mu ,sigma):
        probability = (1/(sigma * math.sqrt(2*math.pi)))*math.exp(-((x-mu)^2)/(2*(sigma^2)))
        return probability


    def computeParticleD(self, scale, muD, muTheta, sigma, x):
        # for each particle
        self.findPDF(x,muD,sigma)
        # need to find the probability of a particle at each position given a sensor reading at that postition (use the map.closest_distance)


        # for i in range(0, 2,1):
        #     d = np.random.normal(muD, scale, sigma)
        #     theta = np.random.normal(muTheta, scale, sigma)
        #     # generate x and y positions
        #     initialX = self.particles[i][0]
        #     print("initial theta", muTheta)
        #
        #     initialY = self.particles[i][1]
        #     initialZ = self.particles[i][2]
        #     initialTheta = self.particles[i][3]
        #     value = d * math.cos(initialTheta + theta)
        #     value2 = d * math.cos(initialTheta + theta)
        #
        #
        #     self.particles[i][0] = initialX + value #d*np.cos(initialTheta + theta)
        #     self.particles[i][1] = initialY + value2 #d*np.cos(initialTheta + theta)
        #     self.particles[i][2] = initialZ
        #     self.particles[i][3] = theta



    def recieveCommand(self, _muTheta, _muDistance):
        # 90 = turn right, -90 = turn left, 0 = move forward
        self.muTheta = _muTheta
        self.muD = _muDistance
        print("recieved command")
        self.commandGiven = True
        self.computeParticleD(self.scale, self.muD, self.muTheta, self.sigma)