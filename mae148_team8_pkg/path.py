import pickle
import math
import logging
import pathlib
import numpy
import time

class CTE(object):

    def __init__(self, look_ahead=1, look_behind=1, num_pts=None) -> None:
        self.num_pts = num_pts
        self.look_ahead = look_ahead
        self.look_behind = look_behind

    #
    # Find the index of the path element with minimal distance to (x,y).
    # This prefers the first element with the minimum distance if there
    # are more then one.
    #
    def nearest_pt(self, path, x, y, from_pt=0, num_pts=None):
        from_pt = from_pt if from_pt is not None else 0
        num_pts = num_pts if num_pts is not None else len(path)
        num_pts = min(num_pts, len(path))
        if num_pts < 0:
            logging.error("num_pts must not be negative.")
            return None, None, None

        min_pt = None
        min_dist = None
        min_index = None
        for j in range(num_pts):
            i = (j + from_pt) % len(path)
            p = path[i]
            d = dist(p[0], p[1], x, y)
            if min_dist is None or d < min_dist:
                min_pt = p
                min_dist = d
                min_index = i
        return min_pt, min_index, min_dist


    # TODO: update so that we look for nearest two points starting from a given point
    #       and up to a given number of points.  This will speed things up
    #       but more importantly it can be used to handle crossing paths.
    def nearest_two_pts(self, path, x, y):
        if path is None or len(path) < 2:
            logging.error("path is none; cannot calculate nearest points")
            return None, None

        distances = []
        for iP, p in enumerate(path):
            d = dist(p[0], p[1], x, y)
            distances.append((d, iP, p))
        distances.sort(key=lambda elem : elem[0])

        # get the prior point as start of segment
        iA = (distances[0][1] - 1) % len(path)
        a = path[iA]

        # get the next point in the path as the end of the segment
        iB = (iA + 2) % len(path)
        b = path[iB]
        
        return a, b

    def nearest_waypoints(self, path, x, y, look_ahead=1, look_behind=1, from_pt=0, num_pts=None):
        """
        Get the path elements around the closest element to the given (x,y)
        :param path: list of (x,y) points
        :param x: horizontal coordinate of point to check
        :param y: vertical coordinate of point to check
        :param from_pt: index start start search within path
        :param num_pts: maximum number of points to search in path
        :param look_ahead: number waypoints to include ahead of nearest point.
        :param look_behind: number of waypoints to include behind nearest point.
        :return: index of first point, nearest point and last point in nearest path segments
        """
        if path is None or len(path) < 2:
            logging.error("path is none; cannot calculate nearest points")
            return None, None

        if look_ahead < 0:
            logging.error("look_ahead must be a non-negative number")
            return None, None
        if look_behind < 0:
            logging.error("look_behind must be a non-negative number")
            return None, None
        if (look_ahead + look_behind) > len(path):
            logging.error("the path is not long enough to supply the waypoints")
            return None, None

        _pt, i, _distance = self.nearest_pt(path, x, y, from_pt, num_pts)

        # get  start of segment
        a = (i + len(path) - look_behind) % len(path)

        # get the end of the segment
        b = (i + look_ahead) % len(path)

        return a, i, b

    def nearest_track(self, path, x, y, look_ahead=1, look_behind=1, from_pt=0, num_pts=None):
        """
        Get the line segment around the closest point to the given (x,y)
        :param path: list of (x,y) points
        :param x: horizontal coordinate of point to check
        :param y: vertical coordinate of point to check
        :param from_pt: index start start search within path
        :param num_pts: maximum number of points to search in path
        :param look_ahead: number waypoints to include ahead of nearest point.
        :param look_behind: number of waypoints to include behind nearest point.
        :return: start and end points of the nearest track and index of nearest point
        """

        a, i, b = self.nearest_waypoints(path, x, y, look_ahead, look_behind, from_pt, num_pts)

        return (path[a], path[b], i) if a is not None and b is not None else (None, None, None)

    def run(self, path, x, y, from_pt=None):
        """
        Run cross track error algorithm
        :return: cross-track-error and index of nearest point on the path
        """
        cte = 0.
        i = from_pt

        a, b, i = self.nearest_track(path, x, y, 
                                     look_ahead=self.look_ahead, look_behind=self.look_behind, 
                                     from_pt=from_pt, num_pts=self.num_pts)
        
        if a and b:
            logging.info(f"nearest: ({a[0]}, {a[1]}) to ({x}, {y})")
            a_v = Vec3(a[0], 0., a[1])
            b_v = Vec3(b[0], 0., b[1])
            p_v = Vec3(x, 0., y)
            line = Line3D(a_v, b_v)
            err = line.vector_to(p_v)
            sign = 1.0
            cp = line.dir.cross(err.normalized())
            if cp.y > 0.0 :
                sign = -1.0
            cte = err.mag() * sign            
        else:
            logging.info(f"no nearest point to ({x},{y}))")
        return cte, i

class Vec3(object):
    def __init__(self, x=0.0, y=0.0, z=0.0):
        self.x = x
        self.y = y
        self.z = z

    def __add__(self, other):
        return self.add(other)

    def __sub__(self, other):
        return self.subtract(other)

    def __mul__(self, other):
        return self.multiply(other)

    def __div__(self, other):
        return self.multiply(other.reciprocal())

    def __neg__(self):
        return self.scaled(-1.0)

    def __iadd__(self, other): #+= other
        self = self.add(other)
        return self

    def mag(self):
        return math.sqrt(self.x * self.x + self.y * self.y + self.z * self.z)
        
    def scale(self, s):
        self.x *= s
        self.y *= s
        self.z *= s
        return self

    def scaled(self, s):
        r = Vec3()
        r.x = self.x * s
        r.y = self.y * s
        r.z = self.z * s
        return r
        
    def normalize(self):
        m = self.mag()
        self.scale(1.0 / m)
        return self

    def normalized(self):
        m = self.mag()
        v = Vec3(self.x, self.y, self.z)
        if m > 0:
            v.scale(1.0 / m)
        return v

    def subtract(self, v):
        r = Vec3()
        r.x = self.x - v.x
        r.y = self.y - v.y
        r.z = self.z - v.z
        return r
        
    def add(self, v):
        r = Vec3()
        r.x = self.x + v.x
        r.y = self.y + v.y
        r.z = self.z + v.z
        return r
        
    def multiply(self, v):
        r = Vec3()
        r.x = self.x * v.x
        r.y = self.y * v.y
        r.z = self.z * v.z
        return r
        
    def dot(self, v):
        return (self.x * v.x + self.y * v.y + self.z * v.z)
        
    def cross(self, v):
        r = Vec3()
        r.x = (self.y * v.z) - (self.z * v.y)
        r.y = (self.z * v.x) - (self.x * v.z)
        r.z = (self.x * v.y) - (self.y * v.x)
        return r
    
    def dist(self, v):
        r = self.subtract(v)
        return r.mag()

    def reciprocal(self):
        r = Vec3()
        if(self.x != 0.0):
            r.x = 1.0 / self.x
        if(self.y != 0.0):
            r.y = 1.0 / self.y
        if(self.z != 0.0):
            r.z = 1.0 / self.z
        return r
        
    def unit_angle(self, v):
        #note! requires normalized vectors as input
        return math.acos(self.dot(v))
 
class Line3D(object):

    def __init__(self, a, b):
        self.origin = a
        self.dir = a - b
        self.dir.normalize()

    def vector_to(self, p):
        delta = self.origin - p
        dot = delta.dot(self.dir)
        return self.dir.scaled(dot) - delta

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
    
def dist(x1, y1, x2, y2):
    return math.sqrt(math.pow(x2 - x1, 2) + math.pow(y2 - y1, 2))

