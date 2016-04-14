import scipy.stats

import lab10_map
import math
import random
import numpy as np# import scipy

class Particle:
    def __init__(self):
        self.x = 0
        self.y = 0
        self.z = 0.1
        self.theta = 0
        self.xTplusOne = 0
        self.yTplusOne = 0
        self.actualWeight = 1/100
        self.weight = 1/100
        self.previousProbabiltiy  = 1/100

class ParticleFilter:
    def __init__(self):
        self.commandGiven = False
        self.map = lab10_map.Map("lab10.map")
        self.sigma = math.sqrt(0.05)
        self.distanceVariance = math.sqrt(0.05)  #distance variance is 5 cm
        self.directionVariance = math.sqrt(5)    #direction variance is 5 degrees
        self.scale = 100

        self.randomNumbers = random.sample(range(300), 200)
        self.randomTheta = random.sample(range(360), 200)

        # for i in range(0, 100, 1):
        #     self.randomTheta[i] = math.radians(self.randomTheta[i])

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
    # def findPDF(self, x, mu, sigma):
    #     # print(sp.stats.norm.pdf(x, mu, sigma))
    #     probability = (1 / (sigma * math.sqrt(2 * math.pi))) * math.pow(math.exp(-(x - mu)), 2) / (
    #         2 * math.pow(sigma, 2))
    #     return probability

    # def computeParticleD(self, scale, muD, muTheta, sigma, x, sensorReading, weightSum):
    #     conditional_probabilty = self.findPDF(x, sensorReading, sigma)
    #     # print("conditional prob = ", conditional_probabilty, ", previous probability = ", self.previousProbability)
    #     weight = conditional_probabilty * self.previousProbability
    #     # self.previousProbability = conditional_probabilty
    #     return weight / weightSum
    #


    # need to find the probability of a particle at each position given a sensor reading at that postition (use the map.closest_distance)

    def recieveCommand(self, _muTheta, _muDistance, sensorReading):
        # 90 = turn right, -90 = turn left, 0 = move forward

        print("recieved command")
        weightSum = 0


        # move particles
        for i in range(0, self.numOfParticles,1):

            # generate noise for theta using the normal random variable sample
            for j in range(0,1000,1):
                thetaNoise = np.random.normal(0, self.directionVariance, 1)

            # generate noise for distance using the normal random variable sample
            for j in range(0,1000,1):
                distanceNoise = np.random.normal(0, self.distanceVariance, 1)

            # Theta prime
            actualTheta = _muTheta + thetaNoise

            # Distance prime
            actualDistance = _muDistance + distanceNoise

            self.particles[i].theta = actualTheta
            self.particles[i].z = 0.1

            # Xt+1 = Xt + D`cos(theta`)
            xPossible = self.particles[i].x + actualDistance * np.cos(self.particles[i].theta)
            # xPossible = self.particles[i].x + np.log(actualDistance * np.cos(self.particles[i].theta))
            # check for map boundaries... Need to implement method to prevent all obstacles than just outer boundaries:
            if xPossible < 0.0:
                xPossible = 0.0
            elif xPossible > 3.0:
                xPossible = 3.0
            # self.particles[i].xTplusOne = xPossible

            self.particles[i].x = xPossible

            # Yt+1 = Yt + D`sin(theta`)
            yPossible = self.particles[i].y + actualDistance * np.sin(self.particles[i].theta)
            # yPossible = self.particles[i].y + np.log(actualDistance * np.sin(self.particles[i].theta))

            if yPossible < 0.0:
                yPossible = 0.0
            elif yPossible > 3.0:
                yPossible = 3.0
            # self.particles[i].yTplusOne = yPossible

            self.particles[i].y = yPossible

            vLocation = self.map.closest_distance((self.particles[i].x, self.particles[i].y), self.particles[i].theta)

            # self.particles[i].weight = scipy.stats.norm.pdf(sensorReading, vLocation, self.sigma) * self.particles[i].previousProbabiltiy
            self.particles[i].weight = np.log(scipy.stats.norm.pdf(sensorReading, vLocation, self.sigma) * self.particles[i].previousProbabiltiy)

            weightSum += self.particles[i].weight

        print(weightSum)
        testValue = 0
        del self.particleWeights[:]

        for i in range(0, self.numOfParticles, 1):
            self.particles[i].previousProbabiltiy = self.particles[i].actualWeight
            self.particles[i].actualWeight = self.particles[i].weight / weightSum

            # print(self.particles[i].actualWeight)
            self.particleWeights.append(self.particles[i].actualWeight)
            testValue += self.particles[i].actualWeight
            # print(self.particles[i].previousProbabiltiy)

        print(testValue)
        self.resampleParticles()
            # compute posterior probability

            # # If the particle is literally inside a wall, reduce the distance traveled
            # wallDist = self.map.closest_distance((xPossible, yPossible), self.particles[i].theta)
            # if wallDist < 0.05:
            #     self.particles[i].x = self.particles[i].x + (actualDistance * np.cos(self.particles[i].theta) - 0.1)
            #     self.particles[i].y = self.particles[i].y + (actualDistance * np.sin(self.particles[i].theta) - 0.1)
        # for j in range(0,1000,1):
        #    sensorNoise = np.random.normal(sensorReading, self.sigma, 1)
        #


        # # create particle sensor readings and store into an array
        # for i in range(0, self.numOfParticles, 1):
        #     # self.particles[i].weight = self.map.closest_distance((self.particles[i].x, self.particles[i].y), self.particles[i].the
        #     #                                                      )
        #     # = self.map.closest_distance((self.particles[i].x, self.particles[i].y), self.particles[i].theta))
        #     self.particles[i].sensorReading = self.map.closest_distance((self.particles[i].x, self.particles[i].y),self.particles[i].theta)
        #     # print("temp value", self.particles[i].sensorReading)
        #     # particle_sensor_reading_array.append(self.map.closest_distance((self.particles[i].x, self.particles[i].y), self.particles[i].theta))
        #     # particle_sensor_reading_array.append(tempValue)
        #
        #
        # # calculate the weightSum of all the particles
        # for i in range(0, self.numOfParticles, 1):
        #     weightSum += self.computeParticleD(self.scale, _muDistance, _muTheta, self.sigma, sensorReading,
        #                                        self.particles[i].sensorReading, 1)
        #
        # print("weight sum = ", weightSum)
        #
        # # calculate the actual weights of each particle
        # del self.particleWeights[:]
        # for i in range(0, self.numOfParticles, 1):
        #     actualWeight = self.computeParticleD(self.scale, _muDistance, _muTheta, self.sigma, sensorReading,
        #                                           self.particles[i].sensorReading, weightSum)
        #     self.particles[i].weight = actualWeight
        #     self.particleWeights.append(actualWeight)
        #
        # # print("actual weight sum = ", actualWeight)
        #
        #
        #



    # //Use a roulette wheel to probabilistically duplicate particles with high weights,
    # //and discard those with low weights. A ‘Particle’ is some structure that has
    # //a weight element w. The sum of all w’s in oldParticles should equal 1.
    def resampleParticles(self):
        new_particles = []
        cdf = []
        cdf.append(self.particles[0].actualWeight)

        for i in range(1, len(self.particles), 1):
            cdf.append(cdf[i-1] + self.particles[i].actualWeight)

        i = 0
        u = np.random.uniform(0, 100) * 1/self.numOfParticles
        print(u)
        for j in range(0, len(self.particles), 1):
            while u > cdf[i]:
                if i == 99:
                    break
                i += 1

            p = self.particles[i]
            p.weight = 1/len(self.particles)

            new_particles.append(p)

            u += 1/len(self.particles)
        self.particles = new_particles
        for k in range(0, self.numOfParticles, 1):

            # self.particles[k].x = self.particles[k].xTplusOne
            # self.particles[k].y = self.particles[k].yTplusOne
            print(self.particles[k].x, self.particles[k].y, self.particles[k].theta)
        # self.particles = np.random.choice(self.particles, self.numOfParticles, p = self.particleWeights)

