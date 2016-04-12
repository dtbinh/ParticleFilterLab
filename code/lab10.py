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

    def turnCreate(self, goal_Theta, state, pos):
        print("goal t: ", goal_Theta)
        while True:
            state = self.create.update()
            if state is not None:
                self.odometry.update(state.leftEncoderCounts, state.rightEncoderCounts)
                theta = math.atan2(math.sin(self.odometry.theta), math.cos(self.odometry.theta))
                output_theta = self.pidTheta.update(self.odometry.theta, goal_Theta, self.time.time())
                print("out theta: ", output_theta)
                # improved version 2: fuse with velocity controller
                if pos:
                    self.create.drive_direct(int(output_theta), int(-output_theta))
                else:
                    if goal_Theta < 0:
                        self.create.drive_direct(-int(output_theta), int(output_theta))
                    else:
                        self.create.drive_direct(int(output_theta), -int(output_theta))
                    # self.create.drive_direct(-int(output_theta), int(output_theta))
                # else:
                #     self.create.drive_direct(int(output_theta), -int(output_theta))

                if output_theta < 10:
                    self.create.drive_direct(0, 0)
                    break

    def move(self, motorSleepTime, angle):
        # self.theta = angle

        goal_x = 0
        goal_y = 0.5
        goal_theta = angle
        pos = False

        # Set the destination to be half a meter in front of the create
        if angle == 0:
            goal_x = 0.5
            goal_y = 0
            # self.create.drive_direct(100, 100)
            # self.time.sleep(motorSleepTime)
            # self.create.drive_direct(0, 0)
            # self.particleFilter.recieveCommand(angle, 0.5, self.sonar.get_distance())
        # # Set the destination theta to be 90 degrees
        # elif angle < 0:
        #     pos = False
        #     goal_theta = math.pi / 2
        #     # self.create.drive_direct(50,-50)
        #     # self.time.sleep(motorSleepTime)
        #     # self.create.drive_direct(0,0)
        #     # self.particleFilter.recieveCommand(angle, 0.3, self.sonar.get_distance())
        # # Set the destination theta to be -90 degrees
        # elif angle > 0:
        #     pos = True
        #     goal_theta = -math.pi / 2
        #     print("hey")
            # self.create.drive_direct(-50,50)
            # self.time.sleep(motorSleepTime)
            # self.create.drive_direct(0,0)
            # self.particleFilter.recieveCommand(angle, 0.3, self.sonar.get_distance())

        if angle != 0:

            while True:
                state = self.create.update()
                if state is not None:
                    self.odometry.update(state.leftEncoderCounts, state.rightEncoderCounts)
                    newTheta = math.atan2(goal_y - self.odometry.y, goal_x - self.odometry.x)
                    self.turnCreate(newTheta, state, pos)

                    break
            # self.turnCreate(self.goal_theta, pos)


        else:
            while True:
                state = self.create.update()
                if state is not None:
                    self.odometry.update(state.leftEncoderCounts, state.rightEncoderCounts)
                    goal_theta = math.atan2(goal_y - self.odometry.y, goal_x - self.odometry.x)
                    theta = math.atan2(math.sin(self.odometry.theta), math.cos(self.odometry.theta))
                    output_theta = self.pidTheta.update(self.odometry.theta, goal_theta, self.time.time())

                    distance = math.sqrt(math.pow(goal_x - self.odometry.x, 2) + math.pow(goal_y - self.odometry.y, 2))
                    output_distance = self.pidDistance.update(0, distance, self.time.time())
                    self.create.drive_direct(int(output_theta + output_distance), int(-output_theta + output_distance))
                    if distance < 0.01:
                        False
                        self.create.drive_direct(0, 0)
                        break

        # self.updateDataForParticles()

    def getSensorData(self):
        reading = self.sonar.get_distance()
        self.particleFilter.recieveCommand(0, 0.0, self.sonar.get_distance())
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
                (self.particleFilter.randomNumbers[0] / 100, self.particleFilter.randomNumbers[1] / 100, 0.1),
                self.particleFilter.randomTheta[0])

        # This is an example on how to show particles
        # the format is x,y,z,theta,x,y,z,theta,...
        # self.virtual_create.set_point_cloud(self.data)
        self.virtual_create.set_point_cloud(self.data)
        # This is an example on how to estimate the distance to a wall for the given
        # map, assuming the robot is at (0, 0) and has heading math.pi

        distance = self.map.closest_distance(
                (self.particleFilter.randomNumbers[0] / 100, self.particleFilter.randomNumbers[1] / 100),
                self.particleFilter.randomTheta[0])
        print(self.map.closest_distance(
                (self.particleFilter.randomNumbers[0] / 100, self.particleFilter.randomNumbers[1] / 100),
                self.particleFilter.randomTheta[0]))
        # This is an example on how to detect that a button was pressed in V-REP
        while True:

            b = self.virtual_create.get_last_button()
            if b == self.virtual_create.Button.MoveForward:
                self.move(5, 0)
            elif b == self.virtual_create.Button.TurnLeft:
                self.move(4, math.pi / 2)
            elif b == self.virtual_create.Button.TurnRight:
                self.move(4, -math.pi / 2)
            elif b == self.virtual_create.Button.Sense:
                self.getSensorData()

            self.time.sleep(0.01)
