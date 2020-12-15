"""
sensor_model.py
Provides a SensorModel class to calculate particle weights.
"""
import rospy
from util import getHeading

import math

import laser_trace

PI_OVER_TWO = math.pi/2 

class SensorModel(object):
    def __init__(self):
        # ----- Parameters for particle weight calculation
        self.z_hit = 0.95 		# Default probability if we make a hit
        self.z_short = 0.1 	# Probability of a short reading from 
                               # unexpected obstacle (e.g. person or object)
        self.z_max = 0.05 		# Probability of failure to detect an obstacle,
                               # reported as max range
        self.z_rand = 0.05 	# Random noise on all readings
        
        self.sigma_hit = 0.2 		# Noise on hit
        self.lambda_short = 0.1 	# Noise on short reading
        
        self.box_size = 7
        
        self.ROBOT_SIZE = 5
        self.PATH_SIZE = 7
        self.ROBOT_SIZE_M = 0.3
        self.PATH_SIZE_M = 0.4
        
        self.checked_places = dict({})
        self.checked_paths = dict({})
        
        # Initialise scan parameters to nothing
        self.set_laser_scan_parameters(0, 5.0, 0, 0, 0)
        
    def set_laser_scan_parameters(self,num_readings, scan_range_max,
                                  scan_length, scan_angle_min,
                                  scan_angle_max ):
        """
        Set the parameters for laser scanner that this instance is modeling
        
        :Args:
            | num_readings (int): Number of scan readings to be compared with
                                  predictions when computing particle weights
            | scan_range_max (double): Max range scanner can read
            | scan_length (int) : The number of readings in a complete scan
            | scan_angle_min (double): The min. angle of the scanner
            | scan_angle_max (double): The max. angle of the scanner
        """
        # ----- Laser parameters
        self.scan_range_max = scan_range_max
        num_readings *= self.box_size
        
        # ----- What points to sample the laser at when calculating particle weight
        reading_step = (scan_length - 1) / (num_readings - 1)
        self.reading_points = [(i, scan_angle_min +
                                ((scan_angle_max - scan_angle_min) *
                                 (float(i) / scan_length)))
                               for i in range(0, scan_length, reading_step)]
        
        rospy.loginfo("Sensor model scan parameters set.")
        
    def set_map(self, occupancy_map):
        """
        Set the map that this model should use when calculating expected
        laser readings.
        
        :Args:
            | occupancy_map (sensor_msgs.msg.OccupancyGrid): the map to use
        """
        # ----- Map data
        self.checked_places = dict({})
        self.checked_paths = dict({})
        self. occupancy_map = occupancy_map
        self.map_width = occupancy_map.info.width
        self.map_height = occupancy_map.info.height
        self.map_resolution = occupancy_map.info.resolution # in m per pixel
        self.map_data =  occupancy_map.data 
        self.map_origin_x = ( occupancy_map.info.origin.position.x +
                             (self.map_width / 2.0) * self.map_resolution )
        self.map_origin_y = ( occupancy_map.info.origin.position.y +
                              (self.map_height / 2.0) * self.map_resolution )
        rospy.loginfo("Sensor model map set.")
        
        self.ROBOT_SIZE = int(self.ROBOT_SIZE_M / self.map_resolution)
        self.PATH_SIZE = int(self.PATH_SIZE_M / self.map_resolution)

    def _f_convert_to_grid_pose(self, x, y):
        x = ((x + self.occupancy_map.info.origin.position.x) / self.map_resolution)
        y = ((y + self.occupancy_map.info.origin.position.y) / self.map_resolution)
        return (x, y)

    def convert_to_grid_pose(self, x, y):
        x = int(round((x + self.occupancy_map.info.origin.position.x) / self.map_resolution))
        y = int(round((y + self.occupancy_map.info.origin.position.y) / self.map_resolution))
        return (x, y)
        
    def convert_to_real_pose(self, x, y):
        x = x * self.map_resolution - self.occupancy_map.info.origin.position.x
        y = y * self.map_resolution - self.occupancy_map.info.origin.position.y
        return (x, y)

    def map_free(self, x, y):
        if (x, y) in self.checked_places:
            return self.checked_places[(x, y)]
        else:
            result = self.f(x, y, self.PATH_SIZE)
            self.checked_places[(x, y)] = result
            return result
    
    def path_free(self, x1, y1, x2, y2):
        delta_x = x2 - x1
        delta_y = y2 - y1
        m = int(max(abs(delta_x), abs(delta_y)))
        if m == 0:
            return self.path_free_f(x1, y1, self.ROBOT_SIZE)
        for i in range(m + 1):
            x = int(x1 + (delta_x * i) / m)
            y = int(y1 + (delta_y * i) / m)
            if not self.path_free_f(x, y, self.ROBOT_SIZE):
                return i
        return -1
        
    def path_free_float(self, x1f, y1f, x2f, y2f):
        temp1 = self._f_convert_to_grid_pose(x1f, y1f)
        x1 = temp1[0]
        y1 = temp1[1]
        temp2 = self._f_convert_to_grid_pose(x2f, y2f)
        x2 = temp2[0]
        y2 = temp2[1]
        delta_x = x2 - x1
        delta_y = y2 - y1
        m = (max(abs(delta_x), abs(delta_y)))
        if m == 0:
            return self.path_free_f(int(x1), int(y1), self.ROBOT_SIZE - 2)
        for i in range(int(m) + 1):
            x = int(x1 + (delta_x * i) / m)
            y = int(y1 + (delta_y * i) / m)
            if not self.path_free_f(x, y, self.ROBOT_SIZE - 2):
                return i
        return -1
        
    
    def path_free_f(self, x, y, size):
        if (x, y) in self.checked_paths:
            return self.checked_paths[(x, y)]
        else:
            result = self.f(x, y, size)
            self.checked_paths[(x, y)] = result
            return result
        
    def f(self, x, y, size):
        for i in range (-size, size + 1, 1):
            for j in range (-size, size + 1, 1):
                if i * i + j * j <= size * size:
                    if not self.cell_free(x + i, y + j):
                        return False
        return True
    
    def cell_free(self, x, y):
        index = y * self.map_width
        index += x
        return (self.map_data[index] < 30 and self.map_data[index] >= 0)

    def calc_map_range(self, ox, oy, oa):
        """
        Given a location on the map and a direction, predict the visible
        laser range.
         
        :Args:
            | ox (double): X location of observation
            | oy (double): Y location of observation
            | oa (double): Bearing (from North, in degrees) of the reading
        :Returns:
            | (double) Range (in m) expected to be observed by the laser
        """
        r = laser_trace.map_calc_range(ox, oy, oa, self.map_width,
                                          self.map_height,
                                          self.map_origin_x,
                                          self.map_origin_y,
                                          self.map_resolution,
                                          self.scan_range_max,
                                          self.map_data)
        if r <= self.scan_range_max:
            return r
        else:
            # ----- rospy.logwarn("calc_map_range giving oversized ranges!!")
            return self.scan_range_max
        

   
