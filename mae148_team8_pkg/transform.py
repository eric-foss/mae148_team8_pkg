import time

class PIDController:
    """ Performs a PID computation and returns a control value.
        This is based on the elapsed time (dt) and the current value of the process variable
        (i.e. the thing we're measuring and trying to change).
        https://github.com/chrisspen/pid_controller/blob/master/pid_controller/pid.py
    """

    def __init__(self, p=0, i=0, d=0, debug=False):

        # initialize gains
        self.Kp = p
        self.Ki = i
        self.Kd = d

        # The value the controller is trying to get the system to achieve.
        self.target = 0

        # initialize delta t variables
        self.prev_tm = time.time()
        self.prev_err = 0
        self.error = None
        self.totalError = 0

        # initialize the output
        self.alpha = 0

        # debug flag (set to True for console output)
        self.debug = debug

    def run(self, err):
        curr_tm = time.time()

        self.difError = err - self.prev_err

        # Calculate time differential.
        dt = curr_tm - self.prev_tm

        # Initialize output variable.
        curr_alpha = 0

        # Add proportional component.
        curr_alpha += -self.Kp * err

        # Add integral component.
        curr_alpha += -self.Ki * (self.totalError * dt)

        # Add differential component (avoiding divide-by-zero).
        if dt > 0:
            curr_alpha += -self.Kd * ((self.difError) / float(dt))

        # Maintain memory for next loop.
        self.prev_tm = curr_tm
        self.prev_err = err
        self.totalError += err

        # Update the output
        self.alpha = curr_alpha

        if (self.debug):
            print('PID err value:', round(err, 4))
            print('PID output:', round(curr_alpha, 4))

        return curr_alpha

