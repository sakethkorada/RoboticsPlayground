import numpy as np
import matplotlib.pyplot as plt
import random
from environments.continuous_3d import Environment3D
import math





#2d RTT
class RRT:

    
    class Node:

        def __init__(self, x, y, z, parent=None):
            self.x = x
            self.y = y
            self.z = z
            self.parent = parent

        def get_coords(self):
            return (self.x,self.y, self.z)
    
    def __init__(self, start, goal, env, expand_dist = 2, iterations=1000, stop_early=False, radius_detection=1):
        self.start = start
        self.goal = goal
        self.env = env
        self.expand_dist = expand_dist
        self.iterations = iterations
        self.node_list = []
        self.stop_early = stop_early
        self.radius_detection = radius_detection
        
    def find_nearest_node(self,coords):
        """
        Finds the closest node based on euclidean distance to new point
        Returns the Node.
        
        :param self: 
        :param x_new: x coordinate of new point
        :param y_new: y coordinate of new point
        """
        x_new,y_new,z_new = coords
        return min(self.node_list, key=lambda n: (n.x - x_new)**2 + (n.y - y_new)**2 + (n.z-z_new)**2)

    def calculate_new_node_location(self, near_node_location, random_pt):
        """
        Calculates the location of the new node based on how much distance to expand
        Returns (x,y,z) tuple of new location
        :param self: 
        :param near_node_location: tuple of (x,y,z) coordinats of near vertice
        :param random_pt: tuple of (x,y,z ) coordinates of random pt
        """
        x_near, y_near,z_near = near_node_location
        x_rand, y_rand, z_rand = random_pt

        deltaX = x_rand - x_near 
        deltaY = y_rand - y_near
        deltaZ = z_rand - z_near
        
        dist = math.sqrt((deltaY)**2 + (deltaX)**2 + (deltaZ)**2)
        if(dist == 0):
            return (x_near, y_near,z_near)
        if(dist < self.expand_dist):
            return random_pt
        ratio = self.expand_dist/ dist

        

        x_new = x_near + ratio* deltaX
        y_new = y_near + ratio * deltaY
        z_new = z_near + ratio * deltaZ
        return (x_new, y_new, z_new)
    
    def is_node_collision_free(self, near_node_location, new_node_location):
        """
        Checks if there are any objects that intersect the line between near_node and new_node
        return boolean
        
        :param self: 
        :param near_node_location: tuple of (x,y,z) coordinates of near node location
        :param new_node_location: tuple of (x,y,z) coordinates of new node location
        """
        
        """
        def ccw(A, B, C):
            
            # Returns True if points are in Counter-Clockwise order
            return (C[1] - A[1]) * (B[0] - A[0]) > (B[1] - A[1]) * (C[0] - A[0])

            
        def check_intersect(A, B, C, D):
            # Returns True if line segment AB intersects CD
            return ccw(A, C, D) != ccw(B, C, D) and ccw(A, B, C) != ccw(A, B, D)
        
        for obstacle in self.obstacles:
            # obstacle is a list of (x, y, w, h)
            x0, y0= obstacle[0], obstacle[1]
            x1, y1 = x0 + obstacle[2], y0 + obstacle[3]
            
            #if new point is inside obstacle
            if (x0 <= new_node_location[0] <= x1) and (y0 <= new_node_location[1] <= y1):
                return False
            if(check_intersect(near_node_location, new_node_location, (x0,y0), (x0,y1))):
                return False
            if(check_intersect(near_node_location, new_node_location, (x0,y0), (x1,y0))):
                return False
            if(check_intersect(near_node_location, new_node_location, (x0,y1), (x1,y1))):
                return False
            if(check_intersect(near_node_location, new_node_location, (x1,y1), (x1,y0))):
                return False
        """
        x,y,z = near_node_location
        dx = new_node_location[0] -near_node_location[0]
        dy = new_node_location[1] -near_node_location[1]
        dz = new_node_location[2] -near_node_location[2]
        
        distance = math.sqrt(dx**2 + dy**2 + dz**2)
        if distance == 0: return True
        
        # 2. Normalize direction
        dx /= distance
        dy /= distance
        dz /= distance

        # 3. Check points along the line
        # Check every 1.0 unit (or smaller for better accuracy)
        steps = int(distance / 1.0) 
        
        for i in range(steps + 1):
            check_x = x + dx * i
            check_y = y + dy * i
            check_z = z + dz * i
            
            # Check against ALL obstacles
            for (ox, oy, oz, w, h, d) in self.env.obstacles:
                # Is this point inside the box?
                if (ox <= check_x <= ox + w and 
                    oy <= check_y <= oy + h and 
                    oz <= check_z <= oz + d):
                    return False # Collision!
                    
        return True 

    def calculate_dist_between_nodes(self,node1_location, node2_location):
        node1_x, node1_y, node1_z= node1_location
        node2_x, node2_y, node2_z = node2_location
        
        return math.sqrt((node1_x-node2_x)**2 + (node1_y-node2_y)**2 + (node1_z-node2_z)**2)
    




    #implements RRT algo 
    def planning(self):
        start_node = RRT.Node(self.start[0], self.start[1], self.start[2])
        self.node_list.append(start_node)
        
        for _ in range(self.iterations):
            #generate random point
            random_pt = env.generate_random_pt()

            #find nearest vertice
            nearest_node = self.find_nearest_node(random_pt)
            x_near, y_near,z_near = nearest_node.x, nearest_node.y, nearest_node.z

            #calculate new vertice 
            new_node_location = self.calculate_new_node_location((x_near, y_near,z_near), random_pt)

            #check if joining new vertice and nearest vertice causes collision
            #if no collision, add it
            #else continue
            is_node_valid = self.is_node_collision_free((x_near,y_near,z_near), new_node_location)
            if is_node_valid:
                new_node = RRT.Node(new_node_location[0], new_node_location[1], new_node_location[2], nearest_node)
                self.node_list.append(new_node)
                self.env.draw_line(nearest_node.get_coords(), new_node_location,color='black')
                self.env.update_view()
                self.env.plotter.add_title(f"n={_}")
                    
                if(self.stop_early == True and self.calculate_dist_between_nodes(new_node_location, self.goal) <= self.radius_detection):
                    break         
            else:
                continue 
            
            
        return

    #finds a path
    def find_path(self):

        closest_node = self.find_nearest_node(self.goal)
        while(closest_node.parent != None):
            self.env.draw_line(closest_node.get_coords(), closest_node.parent.get_coords(),color='red',width=3)
            self.env.update_view()
            closest_node = closest_node.parent

        return

# Usage
startPt = (30,30,30)
endPt = (50,40,50)

env = Environment3D(70,70,70)
env.add_random_obstacles(20,3,10)

rtt = RRT(startPt,endPt,env, expand_dist=6, iterations = 500, stop_early=True, radius_detection=6)
rtt.env.setup_env(start_point=startPt,goal_point=endPt)
rtt.planning()
rtt.find_path()

while(True):
    env.update_view()
