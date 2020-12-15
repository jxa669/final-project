"""
@author jake stewart
Pathfinder that uses a_star and turns a path
into directions
"""

import rospy

from geometry_msgs.msg import (PoseWithCovarianceStamped, PoseStamped,
                               Quaternion,  Transform,  TransformStamped )
from tf.msg import tfMessage
from tf import transformations
from nav_msgs.msg import (OccupancyGrid, Path)

import math
import numpy as np
from util import rotateQuaternion, getHeading
from threading import Lock
import heapq
import time
import sensor_model

class AStar(object):
    
    def __init__(self):
        # ----- Initialise fields
        self.initial_pose =  PoseStamped()
        self.occupancy_map = OccupancyGrid()
        self.tf_message = tfMessage()
        self.input_pose =  PoseWithCovarianceStamped()
        
        self._update_lock =  Lock()
        self._sensor_model = sensor_model.SensorModel()
        
        # ----- Parameters
        self.TURN_LENGTH = 2.0
        self.TURN_AMOUND = math.pi * 0.5 * 0.4
        self.GAP_RANGE = 5.0
        self.input_pose.header.frame_id = "/map"
        
        # ----- Sensor model
        self.sensor_model =  sensor_model.SensorModel()

    def initialise_a_star(self, initialpose):
        result = PoseStamped()
        result.pose.position.x = initialpose.pose.pose.position.x
        result.pose.position.y = initialpose.pose.pose.position.y
        result.pose.position.z = initialpose.pose.pose.position.z
        result.pose.orientation.x = initialpose.pose.pose.orientation.x
        result.pose.orientation.y = initialpose.pose.pose.orientation.y
        result.pose.orientation.z = initialpose.pose.pose.orientation.z
        result.pose.orientation.w = initialpose.pose.pose.orientation.w
        return result
        
    def dist(self, x1, y1, x2, y2):
        return math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)
        
    def calculate_directions(self):
        with self._update_lock:
            t = time.time()
            
            poses = self.calculated_path.poses
            
            self.directions = []
            
            direction = getHeading(poses[0].pose.orientation)
            
            initial_direction = self.angle_difference(direction, getHeading(self.initial_pose.pose.orientation))
            
            
            initial_direction %= 2.0 * math.pi
            
            if initial_direction > math.pi:
                initial_direction -= 2.0 * math.pi
                
                
            if initial_direction <= math.pi * 0.25 and initial_direction > math.pi * -0.25:
                self.directions.append("Go ahead of me")
            elif initial_direction <= math.pi * 0.75 and initial_direction > math.pi * 0.25:
                self.directions.append("Go to my left")
            elif initial_direction <= math.pi * -0.25 and initial_direction > math.pi * -0.75:
                self.directions.append("Go to my right")
            else:
                self.directions.append("Go behind me")
            
            distance = 0
            last = 0
            for i in range(len(poses) - 1):
                pose = poses[i]
                new_direction = getHeading(pose.pose.orientation)
                new_turn = self.angle_difference(new_direction, direction)
                direction = new_direction
                rospy.loginfo("Current pose %d, %d", pose.pose.position.x, pose.pose.position.y)
                #TODO count which turn it is
                if (new_turn < -self.TURN_AMOUND):
                    data_points = []
                    #loop from poses[last] to poses[i]
                    for j in range(last, i):
                        x1 = poses[j].pose.position.x
                        y1 = poses[j].pose.position.y
                        x2 = poses[j + 1].pose.position.x
                        y2 = poses[j + 1].pose.position.y
                        delta_x = x2 - x1
                        delta_y = y2 - y1
                        theta = getHeading(poses[j].pose.orientation)
                        m = (max(abs(delta_x), abs(delta_y)))
                        k = 0.0
                        while k <= m:
                            x = (x1 + (delta_x * k) / m)
                            y = (y1 + (delta_y * k) / m)
                            
                            
                            x_a = x + self.GAP_RANGE * math.sin(theta)
                            y_a = y - self.GAP_RANGE * math.cos(theta)
                            #print(str(x_a) + "," + str(y_a))
                            #store distance between obs and path
                            result = self.sensor_model.path_free_float(x, y, x_a, y_a)
                            if result == -1:
                                result = int(self.GAP_RANGE / self.sensor_model.map_resolution)
                            data_points.append(result)
                            k += 0.001
                            
                            
                    number_of_turns = self.get_number_of_turns(data_points)

                    if (distance < 1.5 and number_of_turns <= 1):
                        print(distance)
                        self.directions.append("Then immediately turn right")
                    else:
                        if (number_of_turns == 1):
                            quant = "1st"
                        elif (number_of_turns == 2):
                            quant = "2nd"
                        elif (number_of_turns == 3):
                            quant = "3rd"
                        else:
                            quant = str(number_of_turns) + "th"
                        self.directions.append("Then take the " + quant + " right")
                    distance = 0
                    last = i
                elif (new_turn > self.TURN_AMOUND):
                    data_points = []
                    #loop from poses[last] to poses[i]
                    for j in range(last, i):
                        x1 = poses[j].pose.position.x
                        y1 = poses[j].pose.position.y
                        x2 = poses[j + 1].pose.position.x
                        y2 = poses[j + 1].pose.position.y
                        delta_x = x2 - x1
                        delta_y = y2 - y1
                        theta = getHeading(poses[j].pose.orientation)
                        m = (max(abs(delta_x), abs(delta_y)))
                        k = 0.0
                        while k <= m:
                            x = (x1 + (delta_x * k) / m)
                            y = (y1 + (delta_y * k) / m)
                            
                            
                            x_a = x - self.GAP_RANGE * math.sin(theta)
                            y_a = y + self.GAP_RANGE * math.cos(theta)
                            #print(str(x_a) + "," + str(y_a))
                            #store distance between obs and path
                            result = self.sensor_model.path_free_float(x, y, x_a, y_a)
                            if result == -1:
                                result = int(self.GAP_RANGE / self.sensor_model.map_resolution)
                            data_points.append(result)
                            k += 0.001
                            
                            
                    number_of_turns = self.get_number_of_turns(data_points)

                    if (distance < 1.5 and number_of_turns <= 1):
                        
                        self.directions.append("Then immediately turn left")
                    else:
                        if (number_of_turns == 1):
                            quant = "1st"
                        elif (number_of_turns == 2):
                            quant = "2nd"
                        elif (number_of_turns == 3):
                            quant = "3rd"
                        else:
                            quant = str(number_of_turns) + "th"
                        self.directions.append("Then take the " + quant + " left")
                    distance = 0
                    last = i
                distance += self.dist(pose.pose.position.x, pose.pose.position.y, poses[i + 1].pose.position.x, poses[i + 1].pose.position.y)
                    
                
            new_direction = getHeading(poses[len(poses) - 1].pose.orientation)
            new_turn = self.angle_difference(new_direction, direction)
            if (new_turn < -self.TURN_AMOUND):
                self.directions.append("Continue for " + str(int(distance)) + " meters and finally it will be on your right")
            elif (new_turn > self.TURN_AMOUND):
                self.directions.append("Continue for " + str(int(distance)) + " meters and finally it will be on your left")
            else:
                self.directions.append("Continue for " + str(int(distance)) + " meters and finally it will be in front of you")
        print(self.directions)
        return time.time() - t
        
    def get_number_of_turns(self, data_points):
        
        #import matplotlib.pyplot as plt
        x = np.array(range(len(data_points)))
        y = np.array(data_points)
        #plt.plot(x, y)
        
        z = np.polyfit(x, y, 1)
        p = np.poly1d(z)
        """
        plt.plot(x, p(x), "r--")
        plt.plot(x, 1.2*p(x) + 5.0, "b--")
        plt.plot(x, 0.9*p(x) - 3.0, "b--")
        plt.xlabel('Distance traveled by robot')
        plt.ylabel('Distance between robot and wall')
        plt.show()
        """
        
        number_of_turns = 0
        close = -1
        #-1 is not decided yet, 0 is close to wall, 1 if far from wall
        mean = np.mean(np.array(data_points))
        x = 0
        for k in range(1, len(data_points), 1):
            y = data_points[k]
            if (y <= max(0.9*p(x) - 3.0, 1.0)):
                if (close != 0):
                    if close == 1:
                        print("+1")
                        number_of_turns += 1
                    close = 0
                    print("Low at " + str(x))
            elif (y > 1.2*p(x) + 5.0):
                if (close != 1):
                    print("High at " + str(x))
                    if close == -2:
                        #number_of_turns += 1
                        print("+1")
                    close = 1
            elif close == -1:
                close = -2
            x += 1
            
        #count the number of gaps
        if close <= 0:
            number_of_turns += 1
            print("+1")
                    #print(data_points)
        if number_of_turns == 0:
            return 1
        return number_of_turns
        
    def angle_difference(self, target, start):
        return math.atan2(math.sin(target - start), math.cos(target - start))
        
    def calculate_path_to(self, pose):
            
        temp = self.sensor_model.convert_to_grid_pose(pose.pose.position.x, pose.pose.position.y)
        target_x = temp[0]
        target_y = temp[1]
        target_theta = getHeading(pose.pose.orientation)
        
        rospy.loginfo("Target pose %d, %d", target_x, target_y)
        
        with self._update_lock:
            t = time.time()
            
            temp = self.sensor_model.convert_to_grid_pose(self.initial_pose.pose.position.x, self.initial_pose.pose.position.y)
            
            current_x = temp[0]
            current_y = temp[1]
            path = self.calculate_rough_path(target_x, target_y, current_x, current_y)
            
            
            self.calculated_path = Path()
            self.calculated_path.header.frame_id = "/map"
            
            rospy.loginfo("Reducing path of size %d",len(path))
            
            i = 0
            current_max = len(path)
            while i < len(path) - 1:
                last = len(path) - 1
                while last >= i:
                    x1 = path[i][0]
                    y1 = path[i][1]
                    x2 = path[last][0]
                    y2 = path[last][1]
                    
                    if (self.sensor_model.path_free(x1, y1, x2, y2) == -1):
                        
                        theta = self.calculate_theta(x1, y1, x2, y2)
                            
                        self.calculated_path.poses.append(self.convert_to_real(x1, y1, theta))
                        i = last
                        break
                    last -= 1
            
            x2 = path[len(path) - 1][0]
            y2 = path[len(path) - 1][1]
            
            self.calculated_path.poses.append(self.convert_to_real(x2, y2, target_theta))
            
            rospy.loginfo("Reduced path to length %d", len(self.calculated_path.poses))
    
        return time.time() - t
        
        
    def calculate_theta(self, x1, y1, x2, y2):
        temp1 = self.sensor_model.convert_to_real_pose(x1, y1)
        temp2 = self.sensor_model.convert_to_real_pose(x2, y2)
        
        delta_x = temp2[0] - temp1[0]
        delta_y = temp2[1] - temp1[1]
        theta = math.atan2(delta_y, delta_x)
        return theta
        
    def calculate_rough_path(self, target_x, target_y, start_x, start_y): 
        cost_function = lambda x, y: self.dist(x, y, target_x, target_y)
        
        current_x = start_x
        current_y = start_y
        current_cost = 0

        nearest_x = current_x
        nearest_y = current_y
        nearest_dist = cost_function(current_x, current_y)
        nearest_cost = current_cost
        
        queue = []
        hashmap = dict({})
        touched = dict({})
        
        heapq.heappush(queue, (0 + cost_function(current_x, current_y), 0, current_x, current_y))
        touched[(current_x, current_y)] = 0
    
        while cost_function(current_x, current_y) > 0.1:
            #print(len(queue))
            if len(queue) == 0:
                rospy.loginfo("Couldn't find target using nearest point %d %d %d", nearest_x, nearest_y, nearest_cost)
                current_x = nearest_x
                current_y = nearest_y
                current_cost = nearest_cost
                break
            current_pose = heapq.heappop(queue)
            current_cost = current_pose[1]
            current_x = current_pose[2]
            current_y = current_pose[3]
            
            while (current_x, current_y) in hashmap:
                current_pose = heapq.heappop(queue)
                current_cost = current_pose[1]
                current_x = current_pose[2]
                current_y = current_pose[3]
                if cost_function(x, y) < nearest_dist:
                    nearest_x = current_x
                    nearest_y = current_y
                    nearest_cost = current_cost
                    nearest_dist = cost_function(current_x, current_y)
                
            #rospy.loginfo("Evaluating pose %d, %d, %d", current_x, current_y, current_cost)
            for i in range(-1, 2, 1):
                for j in range(-1, 2, 1):
                    if (i == 0 and j == 0):
                        continue
                    x = current_x + i
                    y = current_y + j
                    if self.sensor_model.map_free(x, y) and not (x, y) in hashmap:
                        cost = current_cost + self.dist(0, 0, i, j)
                        if (x, y) in touched:
                            if touched[(x, y)] > cost:
                                touched[(x, y)] = cost
                                heapq.heappush(queue, (cost + cost_function(x, y), cost, x, y))
                        else:
                            heapq.heappush(queue, (cost + cost_function(x, y), cost, x, y))
                            touched[(x, y)] = cost
                    
            hashmap[current_x, current_y] = current_cost
                
    
        path = []
        path.insert(0, (current_x, current_y))
        rospy.loginfo("Adding pose %d, %d, %d", current_x, current_y, current_cost)
        
        while current_cost > 0:
            for i in range(-1, 2, 1):
                for j in range(-1, 2, 1):
                    if (i != 0 or j != 0):
                        x = current_x + i
                        y = current_y + j
                        if ((x, y) in hashmap) and self.float_equals(hashmap[(x, y)], current_cost - self.dist(0, 0, i, j)):
                            #rospy.loginfo("Adding pose %d, %d, %d", x, y, current_cost)
                            current_x = x
                            current_y = y
                            path.insert(0, (current_x, current_y))
                            current_cost = hashmap[(x, y)]
                            break;
        return path

    def float_equals(self, x, y):
        return abs(x - y) < 0.01

    def convert_to_real(self, x, y, theta):
        n = PoseStamped()
        temp = self.sensor_model.convert_to_real_pose(x, y)
        n.pose.position.x = temp[0]
        n.pose.position.y = temp[1]
        n.pose.orientation = rotateQuaternion(Quaternion(w=1.0), theta)
        n.header.frame_id = "/map"
        rospy.loginfo("Adding pose %f, %f, %f", temp[0], temp[1], theta)
        return n
    
    def set_initial_pose(self, pose):
        """ Initialise start pose """
        with self._update_lock:
            self.input_pose.pose = pose.pose
            # ----- Estimated pose has been set, so we should now reinitialise the 
            # ----- particle cloud around it
            rospy.loginfo("Got pose. Calling initialise_a_star().")
            self.initial_pose = self.initialise_a_star(self.input_pose)
            self.initial_pose.header.frame_id = "/map"
    
    def set_map(self, occupancy_map):
        """ Set the map for pathfinding """
        with self._update_lock:
            self.occupancy_map = occupancy_map
            
            self.map_width = occupancy_map.info.width
            self.map_height = occupancy_map.info.height
            self.map_resolution = occupancy_map.info.resolution # in m per pixel
            self.map_data =  occupancy_map.data 
            self.map_origin_x = ( occupancy_map.info.origin.position.x +
                                 (self.map_width / 2.0) * self.map_resolution )
            self.map_origin_y = ( occupancy_map.info.origin.position.y +
                                  (self.map_height / 2.0) * self.map_resolution )
                                  
            self.sensor_model.set_map(occupancy_map)
            # ----- Map has changed, so we should reinitialise the particle cloud
            rospy.loginfo("Pathfinding got map. (Re)initialising.")
            
            self.initial_pose = self.initialise_a_star(self.input_pose)
            self.initial_pose.header.frame_id = "/map"
