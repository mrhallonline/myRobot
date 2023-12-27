from robot import Robot
from time import sleep
robot = Robot()
class ObstacleAvoidingBehavior():
    """Simple obstacle avoiding"""
    def __init__(self, the_robot):
        self.robot = the_robot
        self.speed =60

    def get_motor_speed(self, distance):
        if distance < 0.2:
            return -self.speed
        else:
            return self.speed
        
    def get_speeds(self, nearest_distance):
        if nearest_distance >= 1.0:
            nearest_speed =self.speed
            furthest_speed = self.speed
            delay = 100
        elif nearest_distance > 0.5:
            nearest_speed =self.speed
            furthest_speed = self.speed * 0.8
            delay = 100
        elif nearest_distance > 0.2:
            nearest_speed =self.speed
            furthest_speed = self.speed * 0.6
            delay = 100
        elif nearest_distance > 0.1:
            nearest_speed = -self.speed
            furthest_speed = -self.speed * 0.4
            delay = 100
        else: # collision
            nearest_speed = -self.speed
            furthest_speed = -self.speed * 0.8
            delay = 250
        return nearest_speed, furthest_speed, delay
    
    def run(self):
        self.robot.set_pan(0)
        self.robot.set_tilt(0)
        while True:
            left_distance = self.robot.read_left_sensor(50)
            right_distance = self.robot.read_right_sensor(50)
            self.display_state(left_distance, right_distance)
            print("Left: {l:.2f}, Right: {r:.2f}".format(l=left_distance, r=right_distance))
            nearest_speed, furthest_speed, delay = self.get_speeds(min(left_distance, right_distance))
            print(f"Distances: 1{left_distance:.2f}, r{right_distance:.2f} .Delay: {delay}")

            if left_distance < right_distance:
                self.robot.set_left(nearest_speed)
                self.robot.set_right(furthest_speed)  
            else:              
                self.robot.set_right(nearest_speed)
                self.robot.set_left(furthest_speed)  
            sleep(delay * 0.001)


behavior = ObstacleAvoidingBehavior(robot)
behavior.run()