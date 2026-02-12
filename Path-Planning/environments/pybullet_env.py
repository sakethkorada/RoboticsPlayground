import pybullet as p
import pybullet_data
import time
import random

class PyBulletEnv:
    def __init__(self, width=50, height=50, depth=50):
        self.width = width
        self.height = height
        self.depth = depth
        self.obstacles = []
        
        # 1. Connect to Physics Server
        # p.GUI makes a window pop up. p.DIRECT runs in background (faster).
        self.client = p.connect(p.GUI)
        
        # 2. Set up basic scene
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0, 0, -9.8)
        
        # Load a floor plane so things don't fall into the void
        self.plane_id = p.loadURDF("plane.urdf")
        
        # Move camera to get a good view
        p.resetDebugVisualizerCamera(cameraDistance=60, cameraYaw=45, cameraPitch=-45, cameraTargetPosition=[25, 25, 0])

    def add_random_obstacles(self, count=10):
        for _ in range(count):
            # Random size (half-extents)
            w, h, d = random.uniform(1, 5), random.uniform(1, 5), random.uniform(1, 5)
            
            # Random position
            x = random.uniform(0, self.width)
            y = random.uniform(0, self.height)
            z = random.uniform(0, self.depth) # Keep low to ground
            
            self._create_box(x, y, z, w, h, d)
            
            # Store nicely for your RRT to read (x,y,z, w,h,d)
            # Note: PyBullet sizes are "half-extents" (radius), so we multiply by 2 for width
            self.obstacles.append((x - w, y - h, z - d, w*2, h*2, d*2))

    def mark_point(self, position, color=[0, 1, 0], radius=1.0, text=None):
        """
        Creates a visual ghost sphere at a location.
        color: [R, G, B] (0-1)
        radius: size of blob
        text: Optional label above the blob
        """
        # 1. Create the Visual "Blob" (Sphere)
        # Note: We append [0.7] to the color for Alpha (Transparency)
        rgba = color + [0.7] 
        visual_shape = p.createVisualShape(p.GEOM_SPHERE, radius=radius, rgbaColor=rgba)
        
        # 2. Create the Body (Mass=0 means Static/Ghost)
        # baseCollisionShapeIndex=-1 means NO collision (robot can drive through it)
        p.createMultiBody(
            baseMass=0, 
            baseVisualShapeIndex=visual_shape, 
            baseCollisionShapeIndex=-1, 
            basePosition=position
        )
        
        # 3. Optional: Add Text Label
        if text:
            p.addUserDebugText(text, [position[0], position[1], position[2] + radius], 
                               textSize=1.5, textColorRGB=color)

    def _create_box(self, x, y, z, w, h, d):
        """Helper to create a physical box in PyBullet"""
        visual_shape = p.createVisualShape(p.GEOM_BOX, halfExtents=[w, h, d], rgbaColor=[0.6, 0.4, 0.2, 1])
        collision_shape = p.createCollisionShape(p.GEOM_BOX, halfExtents=[w, h, d])
        
        p.createMultiBody(
            baseMass=0, # 0 mass = static (won't fall)
            baseCollisionShapeIndex=collision_shape,
            baseVisualShapeIndex=visual_shape,
            basePosition=[x, y, z]
        )
    def generate_random_pt(self):
        x = random.randint(0, self.width)
        y = random.randint(0, self.height)
        z = random.randint(0, self.depth)
        return (x,y,z)
        

    def draw_line(self, point_a, point_b, color=[0, 0, 1], width=2):
        """
        Draws a debug line. 
        Returns the 'line_id' (like actor) so we can remove it later.
        Color must be a list of 3 floats [R, G, B] (0 to 1).
        """
        # Convert string colors to RGB if necessary, or just stick to RGB
        if color == 'green': color = [0, 1, 0]
        if color == 'red': color = [1, 0, 0]
        if color == 'blue': color = [0, 0, 1]
            
        return p.addUserDebugLine(point_a, point_b, lineColorRGB=color, lineWidth=width)

    def remove_line(self, line_id):
        p.removeUserDebugItem(line_id)

    def update_view(self):
        """
        PyBullet updates automatically, but we sleep to keep the simulation speed readable.
        """
        p.stepSimulation()
        time.sleep(1./240.) # Run at 240Hz