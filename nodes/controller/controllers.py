import numpy as np

class PID(object):
    """PID Controller

    Simple controller based on Dr. Beard's ECEn 483 Control Book, ch 13.

    Create an instance of this class for every PID loop that you want. Then
    call the update() function periodically (10 Hz).

    kp:                 Proportional gain
    ki:                 Integrator gain
    kd:                 Derivative gain
    limit:              Saturate the output so that it stays below this value (+/-)
    tau:                Constant for band-limited dirty derivative
    integrator_limit:   Don't start integrating until derivative is below this value,
                        i.e., if waveform is moving quickly, don't integrate
    """
    def __init__(self, kp, ki, kd, limit, tau, integrator_limit=0.05):
        super(PID, self).__init__()
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.limit = limit
        self.tau = tau
        self.integrator_limit = integrator_limit

        # Declare persistent/static variables
        self.integrator = 0
        self.xdot = 0
        self.error_d1 = 0
        self.x_d1 = 0
        

    def update(self, x_c, x, Ts, max_error_window=0.425):
        """Update

        Computes the output 'force' to send to the plant so that x_c == x
        """

        # compute the error
        use_window_error = (max_error_window is not 0)
        if use_window_error and abs(x_c - x) > max_error_window:
            error = max_error_window*np.sign(x_c - x)
        else:
            error = x_c - x

        # update derivative of x
        self.xdot = self._tustin_derivative(x, Ts)

        # update integral of error
        if self.ki and abs(self.xdot)<self.integrator_limit:
            self.integrator = self._tustin_integral(error, Ts)

        # update delayed variables for next time
        self.error_d1 = error
        self.x_d1 = x

        # compute the PID control signal
        u_unsat = self.kp*error + self.ki*self.integrator - self.kd*self.xdot;
        u = self._sat(u_unsat)

        # more integrator anti-windup
        if self.ki:
            self.integrator = self.integrator + Ts/self.ki*(u-u_unsat)

        return u


    def _sat(self, val):
        out = 0

        if val > self.limit:
            out = self.limit
        elif val < -self.limit:
            out = -self.limit
        else:
            out = val

        return out

    def _tustin_derivative(self, x, Ts):
        """Tustin Derivative
        Using the Tustin Approximation (Bilinear Transform), compute
        a discrete dirty-derivative.
        """
        return (2*self.tau-Ts)/(2*self.tau+Ts)*self.xdot + 2/(2*self.tau+Ts)*(x-self.x_d1)

    def _tustin_integral(self, x, Ts):
        """Tustin Integral
        Using the Tustin approximation (Bilinear Transform), compute
        a discrete integral.
        """
        return self.integrator + (Ts/2)*(x+self.x_d1)
