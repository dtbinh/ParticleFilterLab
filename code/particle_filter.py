import scipy.stats

import lab10_map
import math
import random
import numpy as np# import scipy

class Particle:
    def __init__(self):
        num = 10
        self.x = 0
        self.y = 0
        self.z = 0.1
        self.theta = 0
        # self.xTplusOne = 0
        # self.yTplusOne = 0
        self.actualWeight = 1/num
        self.weight = 1/num
        self.previousProbabiltiy  = 1/num

class ParticleFilter:
    def __init__(self):
        self.commandGiven = False
        self.map = lab10_map.Map("lab10.map")
        print("distance from wall = ", self.map.closest_distance([0.5, 0.5], math.pi*0.0))

        self.sigma = 0.05
        self.distanceVariance = 0.1 #math.sqrt(0.05)  #distance variance is 5 cm
        self.directionVariance = 0.1 #math.sqrt(0.05)    #direction variance is 5 degrees
        self.scale = 100

        self.randomNumbers = random.sample(range(300), 200)
        self.randomTheta = random.sample(range(360), 200)
        self.numOfParticles = 10

        # for i in range(0, self.numOfParticles, 1):
        #     self.randomTheta[i] = math.radians(self.randomTheta[i])

        self.particles = []
        self.particleWeights = []

        for i in range(0, self.numOfParticles*2, 2):
            particle = Particle()
            particle.x = self.randomNumbers[i]/100
            particle.y = self.randomNumbers[i+1]/100
            particle.theta = 0 #math.radians(self.randomTheta[i])
            # print("theta = ", particle.theta)

            # tuple = (, self.randomNumbers[i + 1] / 100, 0.1, self.randomTheta[i])
            self.particles.append(particle)

    # need to find the probability of a particle at each position given a sensor reading at that postition (use the map.closest_distance)


    def moveParticlesForward(self, _muDistance):
        for i in range(0, self.numOfParticles, 1):
            actualDistance = 0
            # generate noise for distance using the normal random variable sample
            for j in range(0, 1000, 1):
                distanceNoise = np.random.normal(0, self.distanceVariance, 1)

            actualDistance = _muDistance + distanceNoise
            # Xt+1 = Xt + D`cos(theta`)
            xPossible = self.particles[i].x + actualDistance * math.cos(self.particles[i].theta)
            # xPossible = self.particles[i].x + np.log(actualDistance * np.cos(self.particles[i].theta))
            # check for map boundaries... Need to implement method to prevent all obstacles than just outer boundaries:
            if xPossible < 0.0:
                xPossible = 0.0
            elif xPossible > 3.0:
                xPossible = 3.0

            self.particles[i].x = xPossible

            # Yt+1 = Yt + D`sin(theta`)
            yPossible = self.particles[i].y + actualDistance * math.sin(self.particles[i].theta)
            # yPossible = self.particles[i].y + np.log(actualDistance * np.sin(self.particles[i].theta))

            if yPossible < 0.0:
                yPossible = 0.0
            elif yPossible > 3.0:
                yPossible = 3.0

            self.particles[i].y = yPossible

    def turnParticles(self,_muTheta):
        for i in range(0, self.numOfParticles, 1):
            actualTheta = 0

            # generate noise for theta using the normal random variable sample
            for j in range(0, 1000, 1):
                thetaNoise = np.random.normal(0, self.directionVariance, 1)

            actualTheta = _muTheta + thetaNoise
            self.particles[i].theta = self.particles[i].theta + actualTheta



    def sensing(self, sensorReading):
        weightSum = 0

        for i in range(0, self.numOfParticles, 1):
            print("x = ", self.particles[i].x, "y = ", self.particles[i].y, "theta = ", self.particles[i].theta)
            vLocation = self.map.closest_distance((self.particles[i].x, self.particles[i].y), self.particles[i].theta)
            print("distance from wall = ", self.map.closest_distance((1,0.5),0))
            self.particles[i].weight = scipy.stats.norm.pdf(sensorReading, vLocation, self.sigma) * self.particles[i].previousProbabiltiy
            # self.particles[i].weight = math.log(scipy.stats.norm.pdf(sensorReading, vLocation, self.sigma) * self.particles[i].previousProbabiltiy)

            weightSum += self.particles[i].weight

        del self.particleWeights[:]

        for i in range(0, self.numOfParticles, 1):
            self.particles[i].previousProbabiltiy = self.particles[i].actualWeight
            self.particles[i].actualWeight = self.particles[i].weight / weightSum
            print("actual weights = ", self.particles[i].actualWeight)
            self.particleWeights.append(self.particles[i].actualWeight)

        self.resampleParticles()





    def recieveCommand(self, _muTheta, _muDistance, sensorReading):
        # 90 = turn right, -90 = turn left, 0 = move forward
        print("recieved command")

        if _muTheta == 0:
            self.moveParticlesForward(_muDistance)

        else:
            self.turnParticles(_muTheta)

        # self.sensing(sensorReading)

        # move particles
        # for i in range(0, self.numOfParticles, 1):
        #     actualTheta = 0
        #     actualDistance = 0
        #     # generate noise for theta using the normal random variable sample
        #     for j in range(0, 1000, 1):
        #         thetaNoise = np.random.normal(0, self.directionVariance, 1)
        #
        #     # generate noise for distance using the normal random variable sample
        #     for j in range(0, 1000, 1):
        #         distanceNoise = np.random.normal(0, self.distanceVariance, 1)
        #     # print("noise distance = ", distanceNoise)
        #
        #     # print("t = ", _muTheta)
        #     # Theta prime
        #     actualTheta = _muTheta + thetaNoise
        #
        #     # Distance prime
        #     actualDistance = _muDistance + distanceNoise
        #
        #     self.particles[i].theta = self.particles[i].theta + actualTheta
        #     # print("theta = ", self.particles[i].theta)
        #     self.particles[i].z = 0.1
        #
        #     # Xt+1 = Xt + D`cos(theta`)
        #     xPossible = self.particles[i].x + actualDistance * math.cos(self.particles[i].theta)
        #     # xPossible = self.particles[i].x + np.log(actualDistance * np.cos(self.particles[i].theta))
        #     # check for map boundaries... Need to implement method to prevent all obstacles than just outer boundaries:
        #     if xPossible < 0.0:
        #         xPossible = 0.0
        #     elif xPossible > 3.0:
        #         xPossible = 3.0
        #     # self.particles[i].xTplusOne = xPossible
        #
        #     self.particles[i].x = xPossible
        #
        #     # Yt+1 = Yt + D`sin(theta`)
        #     yPossible = self.particles[i].y + actualDistance * math.sin(self.particles[i].theta)
        #     # yPossible = self.particles[i].y + np.log(actualDistance * np.sin(self.particles[i].theta))
        #
        #     if yPossible < 0.0:
        #         yPossible = 0.0
        #     elif yPossible > 3.0:
        #         yPossible = 3.0
        #     # self.particles[i].yTplusOne = yPossible
        #
        #     self.particles[i].y = yPossible

            # vLocation = self.map.closest_distance((self.particles[i].x, self.particles[i].y), self.particles[i].theta)
            #
            # self.particles[i].weight = scipy.stats.norm.pdf(sensorReading, vLocation, self.sigma) * self.particles[i].previousProbabiltiy
            # # self.particles[i].weight = math.log(scipy.stats.norm.pdf(sensorReading, vLocation, self.sigma) * self.particles[i].previousProbabiltiy)
            # # print("P    w = ", weightSum)
            # # print(" weight est = ", self.particles[i].weight)
            # weightSum += self.particles[i].weight

            # print("theta new = ", self.particles[i].theta)
        # print("w = ", weightSum)
        # # print(self.particles[0].x, self.particles[0].y, self.particles[0].theta)
        # testValue = 0
        # del self.particleWeights[:]
        #
        # #
        #
        #
        # for i in range(0, self.numOfParticles, 1):
        #     self.particles[i].previousProbabiltiy = self.particles[i].actualWeight
        #     self.particles[i].actualWeight = self.particles[i].weight / weightSum
        #
        #     self.particleWeights.append(self.particles[i].actualWeight)
        #     testValue += self.particles[i].actualWeight
        #     # print("prob = ", self.particles[i].actualWeight)
        #     # print("asdfasdfasd = ", self.particleWeights[i])
        # print("prob sum = ", testValue)
        # self.resampleParticles()


    # //Use a roulette wheel to probabilistically duplicate particles with high weights,
    # //and discard those with low weights. A ‘Particle’ is some structure that has
    # //a weight element w. The sum of all w’s in oldParticles should equal 1.
    def resampleParticles(self):
        # new_particles = []
        # cdf = []
        # cdf.append(self.particles[0].actualWeight)
        # print(len(self.particles))
        # for i in range(1, len(self.particles), 1):
        #     cdf.append(cdf[i-1] + self.particles[i].actualWeight)
        #
        # i = 0
        # u = random.uniform(0.0, 1.0) #* 1/self.numOfParticles
        # print(u)
        # for j in range(0, len(self.particles), 1):
        #     while u > cdf[i]:
        #         if i == (len(self.particles) - 1):
        #             break
        #         i += 1
        #
        #     p = self.particles[i]
        #     p.weight = 1/len(self.particles)
        #
        #     new_particles.append(p)
        #
        #     u += 1/len(self.particles)
        #
        # self.particles = new_particles
        # for k in range(0, self.numOfParticles, 1):
        #     print("actual new weight = ",self.particles[k].actualWeight)
            # self.particles[k].x = self.particles[k].xTplusOne
            # self.particles[k].y = self.particles[k].yTplusOne
            # print(self.particles[k].x, self.particles[k].y, self.particles[k].theta)
        new_particles = np.random.choice(self.particles, self.numOfParticles, p =self.particleWeights)
        del self.particles[:]
        for i in range(0, self.numOfParticles, 1):
            particle = Particle()
            particle.x = new_particles[i].x
            particle.y = new_particles[i].y
            particle.theta = new_particles[i].theta #math.radians(self.randomTheta[i])        #
            particle.actualWeight = 1/self.numOfParticles
            particle.weight = 1/self.numOfParticles
            particle.z = 0.1
            self.previousProbability = 1/self.numOfParticles
            self.particles.append(particle)
        # newSum = 0
        # for i in range(0,self.numOfParticles, 1):
        #     newSum = newSum + self.particles[i].actualWeight
        #
        #
        # #normalize
        # for i in range(0,self.numOfParticles, 1):
        #     self.particles[i].actualWeight = (self.particles[i].actualWeight)/newSum



        # print("length = ",len(self.particles))

