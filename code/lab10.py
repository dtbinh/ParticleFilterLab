import lab10_map
import math
import particle_filter
import odometry


class Run:
    def __init__(self, factory):
        """Constructor.

        Args:
            factory (factory.FactoryCreate)
        """
        self.create = factory.create_create()
        self.time = factory.create_time_helper()
        self.servo = factory.create_servo()
        self.sonar = factory.create_sonar()
        # Add the IP-address of your computer here if you run on the robot
        self.virtual_create = factory.create_virtual_create()
        self.map = lab10_map.Map("lab10.map")

    def moveForward(self, motorSleepTime):
        print("Forward pressed!")
        self.create.drive_direct(100,100)
        self.time.sleep(motorSleepTime)
        self.create.drive_direct(0,0)


    def turnRight(self, motorSleepTime):
        print("Turn Right pressed!")

        self.create.drive_direct(-50,50)
        self.time.sleep(motorSleepTime)
        self.create.drive_direct(0,0)

    def turnLeft(self, motorSleepTime):
        print("Turn Left pressed!")
        self.create.drive_direct(50,-50)
        self.time.sleep(motorSleepTime)
        self.create.drive_direct(0,0)

    def getSensorData(self):
        print("Sense pressed!")


    def run(self):
        # This is an example on how to visualize the pose of our estimated position
        # where our estimate is that the robot is at (x,y,z)=(0.5,0.5,0.1) with heading pi
        self.virtual_create.set_pose((0.5, 0.5, 0.1), math.pi)

        # This is an example on how to show particles
        # the format is x,y,z,theta,x,y,z,theta,...
        data = [0.5, 0.5, 0.1, math.pi/2, 1.5, 1, 0.1, 0]
        self.virtual_create.set_point_cloud(data)

        # This is an example on how to estimate the distance to a wall for the given
        # map, assuming the robot is at (0, 0) and has heading math.pi
        print(self.map.closest_distance((0.5,0.5), math.pi))

        # This is an example on how to detect that a button was pressed in V-REP
        while True:
            b = self.virtual_create.get_last_button()
            if b == self.virtual_create.Button.MoveForward:
                self.moveForward(2)
            elif b == self.virtual_create.Button.TurnLeft:
                self.turnLeft(2)
            elif b == self.virtual_create.Button.TurnRight:
                self.turnRight(2)
            elif b == self.virtual_create.Button.Sense:
                self.getSensorData()

            self.time.sleep(0.01)
