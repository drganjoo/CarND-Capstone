from pid import PID

GAS_DENSITY = 2.858
ONE_MPH = 0.44704


class Controller(object):
    def __init__(self, *args, **kwargs):
        # TODO: Implement
        self.throttle_pid = PID(1,0,0,0,1)

    def control(self, target_velocity, target_angular_velocity, current_velocity, current_angular_velocity, dbw_enabled):
        # TODO: Change the arg, kwarg list to suit your needs
        # Return throttle, brake, steer
        throttle_error = target_velocity - current_velocity

        throttle = self.throttle_pid.step(throttle_error, 1.0 / 50.0)

        steer = 0
        brake = 0

        return throttle
        #return 1., 0., 0.
