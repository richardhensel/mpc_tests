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

    def initialise_pose(self, easting, northing, yaw, vel_x, vel_y):
        self.easting = easting
        self.northing = northing
        self.yaw = yaw
        self.velocity_x = vel_x
        self.velocity_y = vel_y

    def control(self, accelerator_cmd, brake_cmd, steering_cmd):
        self.accelerator_cmd = accelerator_cmd
        self.brake_cmd = brake_cmd
        self.steering_cmd = steering_cmd

        print('Steering angle cmd= ' + str(steering_cmd))

    def update(self, time_delta):

        self.acceleration = (self.accelerator_cmd - self.brake_cmd) * 4.0
        self.steering_angle = self.steering_cmd * 0.4
        print('Steering angle actual= ' + str(self.steering_angle))
        
        # Kinematic bycicle model taken from
        # https://archit-rstg.medium.com/two-to-four-bicycle-model-for-car-898063e87074

        dt = time_delta

        v  = math.sqrt(math.pow(self.velocity_x, 2) + math.pow(self.velocity_y, 2))
        # beta = math.atan(self.velocity_y/self.velocity_x)
        psi = self.yaw
        sigma = self.steering_angle
        lr = abs(self.x_origin_rear - self.x_origin_cog)
        lf = abs(self.x_origin_front - self.x_origin_cog)
        l = lr + lf

        a = self.acceleration
        x = self.easting
        y = self.northing

        beta = math.atan(lr * math.tan(sigma) / l)

        x_dot = v * math.cos(beta + psi)
        y_dot = v * math.sin(beta + psi)
        psi_dot = v * math.sin(beta) / lr
        v_dot = a

        x_new = x + (x_dot * dt)
        y_new = y + (y_dot * dt)
        psi_new = psi + (psi_dot * dt)
        v_new = v + (v_dot * dt)

        beta_new = math.atan(lr * math.tan(sigma) / l)

        # Update the internal states

        self.easting = x_new
        self.northing = y_new
        self.yaw = self.rectify_angle(psi_new)
        self.velocity_x = v_new * math.cos(beta_new)
        self.velocity_y = v_new * math.sin(beta_new)

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
   


