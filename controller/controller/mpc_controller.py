import math

class Model():

    def __init__(self):

        # Control
        self.accelerator_cmd = 0.0
        self.brake_cmd = 0.0
        self.steering_cmd = 0.0

        # Dynamics inputs
        self.acceleration = 0.0
        self.steering_angle = 0.0

        # Pose
        self.easting = 0.0
        self.northing = 0.0
        self.yaw = 0.0
        self.velocity_x = 0.0
        self.velocity_y = 0.0
        self.steering_angle = 0.0

        # Vehicle attributes
        # x is forward
        # y is left
        # z is up

        self.x_origin_front = 3.0
        self.x_origin_rear = -1.0
        self.y_origin_left = 1.0
        self.y_origin_right = -1.0

        self.x_origin_cog = 1.0
        self.y_origin_cog = 0.0

    def update_state(self, easting, northing, yaw, vel_x, vel_y, steering_angle):
        self.easting = easting
        self.northing = northing
        self.yaw = yaw
        self.velocity_x = vel_x
        self.velocity_y = vel_y
        self.steering_angle = steering_angle



    def update(self, time_delta):



    def control(self, accelerator_cmd, brake_cmd, steering_cmd):
        self.accelerator_cmd = accelerator_cmd
        self.brake_cmd = brake_cmd
        self.steering_cmd = steering_cmd

        print('Steering angle cmd= ' + str(steering_cmd))

    def rectify_angle(self, angle):

        # Deal with angles way out of range. 
        while angle >= (2 * math.pi):
            angle -= (2 * math.pi)
        while angle < 0.0:
            angle += (2 * math.pi)

        # Rectify angles to between -pi and pi

        if angle > math.pi:
            angle -= (2 * math.pi)

        if angle < (-1 * math.pi):
            angle += (2 * math.pi) 
            
        return angle

