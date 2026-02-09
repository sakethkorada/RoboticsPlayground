import numpy as np
import pyvista as pv
import random


class Environment3D:
    def __init__(self, width=100, height=100,depth=100):
        self.width = width
        self.height = height
        self.depth = depth
        self.obstacles = [] # List of (x, y, z, w, h, d)
        self.plotter = pv.Plotter()
        
        
       

    def add_random_obstacles(self, count=10,lowerBound=2,upperBound=3):
        
        for _ in range(count):
            
            w, h, d= random.randint(lowerBound,upperBound), random.randint(lowerBound,upperBound),random.randint(lowerBound,upperBound)
            x = random.randint(0, self.width - w)
            y = random.randint(0, self.height - h)
            z = random.randint(0, self.depth -d)
            self.obstacles.append((x, y, z, w,h, d))
        

        
    def get_obstacles(self):
        return self.obstacles   
    
    

    def generate_random_pt(self):
        x = random.randint(0, self.width )
        y = random.randint(0, self.height)
        z = random.randint(0, self.depth)
        return (x,y,z)
    
    
    
        
    

    def setup_env(self, start_point=None, goal_point=None):
        """
        Creates the interactive 3D window.
        """
        
        #create plotter
        self.plotter.set_background('white')
        plotter = self.plotter
        #add bounds
        bounds = pv.Cube(center=(self.width/2, self.height/2, self.depth/2), 
                         x_length=self.width, y_length=self.height, z_length=self.depth)
        plotter.add_mesh(bounds, style='wireframe', color='black', line_width=2, label="Bounds")

        #add obstacles
        for (x, y, z, dx, dy, dz) in self.obstacles:
            center_x = x + dx / 2
            center_y = y + dy / 2
            center_z = z + dz / 2
            
            obstacle = pv.Cube(center=(center_x, center_y, center_z), 
                               x_length=dx, y_length=dy, z_length=dz)
            plotter.add_mesh(obstacle, color='brown', opacity=0.6)

       
        if start_point:
            plotter.add_mesh(pv.Sphere(radius=1, center=start_point), color='green', label="Start")
        
        if goal_point:
            plotter.add_mesh(pv.Sphere(radius=1, center=goal_point), color='red', label="Goal")

        
        plotter.show_grid()
        plotter.add_axes()
        plotter.add_legend()
        
        
        self.plotter.show(interactive_update=True, auto_close=False)

    def draw_line(self, point_a, point_b, color='blue', width=2):
        
        line = pv.Line(point_a, point_b)
        self.plotter.add_mesh(line, color=color, line_width=width)

    def update_view(self):
        self.plotter.update()

if __name__ == "__main__":
    startPt = (5,5,5)
    goalPt = (45,45,45)

    env = Environment3D(50, 50, 50)
    env.add_random_obstacles(10)
    env.setup_env(startPt, goalPt)
    env.draw_line(point_a=startPt,point_b = goalPt,color='black')
    while True:
        env.update_view()