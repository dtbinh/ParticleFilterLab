import lab10_map
import create2
import math
import numpy as np
import particle_filter
import odometry
import pid_controller


class Run:
    def __init__(self, factory):
        """Constructor.

        Args:
            factory (factory.FactoryCreate)
        """

        # self.pidTheta = pid_controller.PIDController(200, 25, 5, [-1, 1], [-200, 200], is_angle=True)
        # self.pidDistance = pid_controller.PIDController(100, 15, 5, [-10, 10], [-200, 200], is_angle=False)
        self.pidTheta = pid_controller.PIDController(300, 5, 50, [-10, 10], [-200, 200], is_angle=True)
        self.pidDistance = pid_controller.PIDController(1000, 0, 50, [0, 0], [-200, 200], is_angle=False)

        self.create = factory.create_create()
        self.time = factory.create_time_helper()
        self.servo = factory.create_servo()
        self.sonar = factory.create_sonar()
        # Add the IP-address of your computer here if you run on the robot
        self.virtual_create = factory.create_virtual_create()
        self.map = lab10_map.Map("lab10.map")
        self.odometry = odometry.Odometry()
        self.theta = 0
        self.particleFilter = particle_filter.ParticleFilter()
        self.data = []

        for i in range(0, self.particleFilter.numOfParticles, 1):
            # self.data = [self.particleFilter.randomNumbers[i]/100, self.particleFilter.randomNumbers[i+1]/100, 0.1, self.particleFilter.randomTheta[i],]
            self.data.append(self.particleFilter.particles[i].x)
            self.data.append(self.particleFilter.particles[i].y)
            self.data.append(self.particleFilter.particles[i].z)
            self.data.append(self.particleFilter.particles[i].theta)

    def turnCreate(self, goalTheta, goal_x, goal_y):
        start_time = self.time.time()
        initialPos = math.sqrt(math.pow(self.odometry.x, 2) + math.pow(self.odometry.y, 2))
        initialTheta = math.atan2(math.sin(self.odometry.theta), math.cos(self.odometry.theta))
        # initialTheta = self.odometry.theta #math.atan2(math.sin(self.odometry.theta), math.cos(self.odometry.theta))
        initX = self.odometry.x
        initY = self.odometry.y
        desTheta = goalTheta
        print ("goal theta = ", initialTheta)
        goalTheta = initialTheta + goalTheta
        print (" new goal theta = ", goalTheta)


        while True:
            state = self.create.update()
            if state is not None:
                self.odometry.update(state.leftEncoderCounts, state.rightEncoderCounts)
                # goalTheta = math.atan2(goal_y - self.odometry.y, goal_x - self.odometry.x)
                # goalTheta = goalTheta - initialTheta

                if desTheta > 0:
                    output_theta = self.pidTheta.update(self.odometry.theta, goalTheta, self.time.time())
                    # print(output_theta)
                    self.create.drive_direct(int(output_theta), int(-output_theta))
                else:
                    output_theta = -self.pidTheta.update(self.odometry.theta, goalTheta, self.time.time())
                    # print(output_theta)
                    self.create.drive_direct(-int(output_theta), int(output_theta))


                distance = math.sqrt(math.pow(goal_y - self.odometry.x, 2) + math.pow(goal_y - self.odometry.y, 2))
                output_distance = self.pidDistance.update(0, distance, self.time.time())
                print(output_theta)
                # if (output_theta < 5 and goalTheta > 0) or (output_theta > 5 and  < 0):
                if output_theta < 1 and output_theta !=200:
                    actualTheta = initialTheta - output_theta
                    actualDistance = initialPos + distance
                    # print("distance = ", output_distance)
                    # print("actual theta = ", math.degrees(actualTheta))
                    self.create.drive_direct(0, 0)
                    # self.particleFilter.recieveCommand(goalTheta, actualDistance, self.sonar.get_distance())
                    break



    # Move create a half meter forward and return distance traveled
    def moveForward(self, goal_x, goal_y, angle):
        start_time = self.time.time()
        initialPos = math.sqrt(math.pow(goal_x - self.odometry.x, 2) + math.pow(goal_y - self.odometry.y, 2))

        while True:
            state = self.create.update()
            if state is not None:
                self.odometry.update(state.leftEncoderCounts, state.rightEncoderCounts)
                goal_theta = math.atan2(goal_y - self.odometry.y, goal_x - self.odometry.x)
                output_theta = self.pidTheta.update(self.odometry.theta, goal_theta, self.time.time())

                # improved version 2: fuse with velocity controller
                distance = math.sqrt(math.pow(goal_x - self.odometry.x, 2) + math.pow(goal_y - self.odometry.y, 2))
                output_distance = self.pidDistance.update(0, distance, self.time.time())
                self.create.drive_direct(int(output_theta + output_distance), int(-output_theta + output_distance))
                if distance < 0.01:
                    actualDistance = initialPos - distance
                    # print(actualDistance)
                    # self.particleFilter.recieveCommand(angle, actualDistance, self.sonar.get_distance())
                    self.create.drive_direct(0, 0)
                    return actualDistance


    def move(self, motorSleepTime, angle):
        # self.theta = angle

        goal_x = self.odometry.x
        goal_y = self.odometry.y
        goal_theta = angle
        pos = False
        distance =0


        # Set the destination to be half a meter in front of the create
        if angle == 0:
            xpos = 0.5 * math.cos(math.atan2(math.sin(self.odometry.theta), math.cos(self.odometry.theta)))
            ypos = 0.5 * math.sin(math.atan2(math.sin(self.odometry.theta), math.cos(self.odometry.theta)))
            # xpos = 0.5 * math.cos(self.odometry.theta)
            # ypos = 0.5 * math.sin(self.odometry.theta)
            # print("l ", math.atan2(math.sin(self.odometry.theta), math.cos(self.odometry.theta)))

            goal_x += xpos
            goal_y += ypos
            self.create.drive_direct(100, 100)
            self.time.sleep(motorSleepTime)
            self.create.drive_direct(0, 0)
            # Just for testing
            # distanceTraveled = self.moveForward(goal_x, goal_y, angle)
            self.particleFilter.recieveCommand(angle, 0.5, self.sonar.get_distance())

        # # Set the destination theta to be 90 degrees
        elif angle > 0:
            startTime = self.time.time()
            self.create.drive_direct(50,-50)
            self.time.sleep(motorSleepTime)

            self.create.drive_direct(0,0)

            # distanceTraveled = self.turnCreate(angle, goal_x, goal_y)


            self.particleFilter.recieveCommand(angle, 0, self.sonar.get_distance())
        # # Set the destination theta to be -90 degrees
        else:
            # startTime = self.time.time()
            self.create.drive_direct(-50,50)
            self.time.sleep(motorSleepTime)

            self.create.drive_direct(0,0)
            # distanceTraveled = self.turnCreate(angle, goal_x, goal_y)
            self.particleFilter.recieveCommand(angle, 0, self.sonar.get_distance())

        self.updateDataForParticles()

    def getSensorData(self):
        reading = self.sonar.get_distance()
        # self.particleFilter.recieveCommand(0, 0.0, self.sonar.get_distance())
        self.particleFilter.sensing(self.sonar.get_distance())
        self.updateDataForParticles()
        print("Sense pressed! Reading is: ", reading)

        # return reading

    def updateDataForParticles(self):
        del self.data[:]
        for i in range(0, self.particleFilter.numOfParticles, 1):
            self.data.append(self.particleFilter.particles[i].x)
            self.data.append(self.particleFilter.particles[i].y)
            self.data.append(self.particleFilter.particles[i].z)
            self.data.append(self.particleFilter.particles[i].theta)

        self.virtual_create.set_pose((self.particleFilter.particles[i].x, self.particleFilter.particles[i].y, 0.1),
                                     self.particleFilter.particles[i].theta)
        self.virtual_create.set_point_cloud(self.data)

    def run(self):
        self.create.start()
        self.create.safe()

        # request sensors
        self.create.start_stream([
            create2.Sensor.LeftEncoderCounts,
            create2.Sensor.RightEncoderCounts,
        ])

        # This is an example on how to visualize the pose of our estimated position
        # where our estimate is that the robot is at (x,y,z)=(0.5,0.5,0.1) with heading pi
        self.virtual_create.set_pose(
                (self.particleFilter.particles[0].x, self.particleFilter.particles[0].y, 0.1),
                self.particleFilter.particles[0].theta)

        # This is an example on how to show particles
        # the format is x,y,z,theta,x,y,z,theta,...
        # self.virtual_create.set_point_cloud(self.data)
        self.virtual_create.set_point_cloud(self.data)
        # This is an example on how to estimate the distance to a wall for the given
        # map, assuming the robot is at (0, 0) and has heading math.pi


        # print(self.particleFilter.particles[0].x, self.particleFilter.particles[0].y, self.particleFilter.particles[0].theta)
        # print(self.map.closest_distance((self.particleFilter.particles[0].x, self.particleFilter.particles[0].y),self.particleFilter.particles[0].theta))
                # (self.particleFilter.randomNumbers[0] / 100, self.particleFilter.randomNumbers[1] / 100),
                # self.particleFilter.randomTheta[0]))
        # This is an example on how to detect that a button was pressed in V-REP
        while True:

            b = self.virtual_create.get_last_button()
            if b == self.virtual_create.Button.MoveForward:
                self.move(5, 0)
            elif b == self.virtual_create.Button.TurnLeft:
                self.move(3.8, math.pi / 2)
            elif b == self.virtual_create.Button.TurnRight:
                self.move(3.8, -math.pi / 2)
            elif b == self.virtual_create.Button.Sense:
                self.getSensorData()

            self.time.sleep(0.01)
