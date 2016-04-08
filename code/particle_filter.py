import lab10_map
import math
import random
from array import *
import numpy as np

class Particle:
    def __init__(self):
        self.x = 0
        self.y = 0
        self.z = 0.1
        self.theta = 0
        self.actualWeight = 0

class ParticleFilter:
    def __init__(self):
        self.commandGiven = False
        self.map = lab10_map.Map("lab10.map")

        self.muTheta = 0
        self.sigma = 0.05
        self.muD = 0.5  # mean
        self.scale = 100
        self.previousProbability = 1 / 36

        self.randomNumbers = random.sample(range(300), 200)
        self.randomTheta = random.sample(range(360), 200)

        self.particles = []
        self.particleWeights = []
        self.numOfParticles = 100

        for i in range(0, self.numOfParticles*2, 2):
            particle = Particle()
            particle.x = self.randomNumbers[i]/100
            particle.y = self.randomNumbers[i+1]/100
            particle.theta = self.randomTheta[i]
            # tuple = (, self.randomNumbers[i + 1] / 100, 0.1, self.randomTheta[i])
            self.particles.append(particle)

    # x = fake sensor reading, mu = actual sensor reading
    def findPDF(self, x, mu, sigma):
        probability = (1 / (sigma * math.sqrt(2 * math.pi))) * math.pow(math.exp(-(x - mu)), 2) / (
            2 * math.pow(sigma, 2))
        return probability

    def computeParticleD(self, scale, muD, muTheta, sigma, x, sensorReading, weightSum):
        # for each particle
        conditional_probabilty = self.findPDF(x, sensorReading, sigma)
        weight = conditional_probabilty * self.previousProbability
        return weight / weightSum



        # need to find the probability of a particle at each position given a sensor reading at that postition (use the map.closest_distance)

    def recieveCommand(self, _muTheta, _muDistance, sensorReading):
        # 90 = turn right, -90 = turn left, 0 = move forward
        self.muTheta = _muTheta
        self.muD = _muDistance
        print("recieved command")
        # self.commandGiven = True
        weightSum = 0
        actualWeight = 0
        particle_sensor_reading_array = []

        # create particle sensor readings and store into an array
        for i in range(0, self.numOfParticles, 1):
            # self.particles[i].weight = self.map.closest_distance((self.particles[i].x, self.particles[i].y), self.particles[i].the
            #                                                      )
            # = self.map.closest_distance((self.particles[i].x, self.particles[i].y), self.particles[i].theta))
            particle_sensor_reading_array.append(
                    self.map.closest_distance((self.particles[i].x, self.particles[i].y), self.particles[i].theta))

        # calculate the weightSum of all the particles
        for i in range(0, self.numOfParticles, 1):
            weightSum += self.computeParticleD(self.scale, self.muD, self.muTheta, self.sigma, sensorReading,
                                               particle_sensor_reading_array[i], 1)

        print("weight sum = ", weightSum)

        # calculate the actual weights of each particle
        del self.particleWeights[:]
        for i in range(0, self.numOfParticles, 1):
            actualWeight = self.computeParticleD(self.scale, self.muD, self.muTheta, self.sigma, sensorReading,
                                                  particle_sensor_reading_array[i], weightSum)
            self.particles[i].weight = actualWeight
            self.particleWeights.append(actualWeight)

        print("actual weight sum = ", actualWeight)

        self.resampleParticles()

    # //Use a roulette wheel to probabilistically duplicate particles with high weights,
    # //and discard those with low weights. A ‘Particle’ is some structure that has
    # //a weight element w. The sum of all w’s in oldParticles should equal 1.
    def resampleParticles(self):
        old_particles = self.particles
    #     # cdf = []
    #     # for i in range(0, 100, 1):
    #     #     new_particles.append(self.particles[i])
    #     #     cdf.append(cdf)
        self.particles = np.random.choice(self.particles, self.numOfParticles, True, p=self.particleWeights)


