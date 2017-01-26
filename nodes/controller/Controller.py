import numpy as np

from controllers import PID

PID_x = None
PID_y = None
PID_theta = None

_set_point = (0, 0, 0)

# A flag to determine whether or not we are at our set point
_arrived = False

# These let us update theta at smaller rates than x and y
_loop_count = 0
_theta_loops = 1 # so every 2 loops do theta controller

velocities = (0, 0, 0)

def init(gains=None):
    global PID_x, PID_y, PID_theta

    xP = 5
    xI = 0
    xD = 0

    yP = 5
    yI = 0
    yD = 0

    thetaP = 3
    thetaI = 0
    thetaD = 0

    PID_x = PID(xP, xI, xD, 2, 0.05, integrator_limit=0.05)
    PID_y = PID(yP, yI, yD, 2, 0.05, integrator_limit=0.05)
    PID_theta = PID(thetaP, thetaI, thetaD, 180, 0.05, integrator_limit=0.05)

def set_commanded_position(x, y, theta):
    """Set Commanded Position

    x_c, y_c, theta_c. These will tell the controller where it wants to go.

    theta_c (degrees) can be given on the interval [0, 360].
    This function can also receive theta from [-360, 0].

    """
    global _set_point, _arrived

    # We aren't there yet!
    _arrived = False

    # Because theta is periodic with 360 degrees
    # This also deals with theta being negative
    theta = theta % 360

    _set_point = (x, y, theta)
    return True

def get_commanded_position():
    return _set_point

def update(time_since_last_update, xhat, yhat, thetahat):
    global velocities, _arrived, _loop_count

    if _arrived:
        # Don't even try
        return (0, 0, 0)

    if PID_x is None or PID_y is None or PID_theta is None:
        # Controller hasn't been properly initialized
        return (0, 0, 0)

    # We've had another motion loop!
    _loop_count = _loop_count + 1

    # Only update theta every _theta_loops times
    if _loop_count == _theta_loops:
        update_theta = True
        _loop_count = 0
    else:
        update_theta = False

    # Break out variables for easy access
    x_c = _set_point[0]
    y_c = _set_point[1]
    theta_c = _set_point[2]

    Ts = time_since_last_update

    # Initialize velocities
    vx = vy = w = 0

    # Only control the positions that aren't 'close'
    if not _close(x_c, xhat):
        vx = PID_x.update(x_c, xhat, Ts)

    if not _close(y_c, yhat):
        vy = PID_y.update(y_c, yhat, Ts)

    if update_theta and not _close(theta_c, thetahat, tolerance=5): # degrees
        # Since the max distance you should ever go is 180 degrees,
        # test to see so that the commanded value is proportional to
        # the error between commanded and actual.
        # Basicaly, this makes going in circles cooler.
        if abs(thetahat-theta_c) > 180:
            if theta_c < thetahat:
                theta_c = theta_c + 360
            else:
                theta_c = theta_c - 360

        w  = PID_theta.update(theta_c, thetahat, Ts, max_error_window=0)

    # Are we there yet?
    _arrived = (vx == 0 and vy == 0 and w == 0 and update_theta)

    velocities = (vx, vy, w)

    return velocities

def _close(a, b, tolerance=0.010):
    return abs(a - b) <= tolerance
