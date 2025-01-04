import rclpy
import numpy as np
from drone_interfaces.srv import SetObstacleParameters, SetBoxParameters, SetCylinderParameters, SetSphereParameters
from rclpy.utilities import ok, get_default_context

class EnvRandomizer():
    def __init__(self, boundary_shape=[20, 20] , max_obstacles=None):
        # Init ros interface
        self.node = None
        if not rclpy.ok():
            rclpy.init(args=None)
        self.__node = rclpy.create_node('env_randomizer')
        self.box_params_cli = self.__node.create_client(SetBoxParameters, 'set_box_parameters')
        self.cylinder_params_cli = self.__node.create_client(SetCylinderParameters, 'set_cylinder_parameters')
        self.sphere_params_cli = self.__node.create_client(SetSphereParameters, 'set_sphere_parameters')
        self.wait_ros_services()

        self.boundary_shape = boundary_shape
        self.obstacles = []
        self.max_obstacles = max_obstacles
        if max_obstacles:
            pass
        else:
            self.generate_simple_obstacles()
            self.randomize_enviroment()

        self.randomize_goal_point()

    def wait_ros_services(self):
        while not self.box_params_cli.wait_for_service(timeout_sec=2.0):
            self.__node.get_logger().info('set_box_parameters service not available, waiting again...')
        while not self.cylinder_params_cli.wait_for_service(timeout_sec=2.0):
            self.__node.get_logger().info('set_cylinder_parameters service not available, waiting again...')
        while not self.sphere_params_cli.wait_for_service(timeout_sec=2.0):
            self.__node.get_logger().info('set_sphere_parameters service not available, waiting again...')
        self.__node.get_logger().info('All services available')

    def set_obstacle_parameters(self, obstacle_dict):
        if obstacle_dict["type"] == "box":
            return self.set_box_parameters(obstacle_dict)
        elif obstacle_dict["type"] == "cylinder":
            return self.set_cylinder_parameters(obstacle_dict)
        elif obstacle_dict["type"] == "sphere":
            return self.set_sphere_parameters(obstacle_dict)
        else:
            self.__node.get_logger().error(f"Obstacle type {obstacle_dict['type']} not recognized.")
            return None
          
    def set_box_parameters(self, box_obstacle_dict):
        box_def = box_obstacle_dict["def"]
        translation = box_obstacle_dict["translation"]
        axis_angle_rotation = box_obstacle_dict["rotation"]
        size = box_obstacle_dict["size"]
        request = SetBoxParameters.Request()
        request.box_def = box_def
        request.translation.x = float(translation[0])
        request.translation.y = float(translation[1])
        request.translation.z = float(translation[2])
        request.rotation.x = float(axis_angle_rotation[0])
        request.rotation.y = float(axis_angle_rotation[1])
        request.rotation.z = float(axis_angle_rotation[2])
        request.rotation.angle = float(axis_angle_rotation[3])
        request.size.x = float(size[0])
        request.size.y = float(size[1])
        request.size.z = float(size[2])
        future = self.box_params_cli.call_async(request)
        rclpy.spin_until_future_complete(self.__node, future)
        return future.result()

    def set_cylinder_parameters(self, cylinder_obstacle_dict):
        cylinder_def = cylinder_obstacle_dict["def"]
        translation = cylinder_obstacle_dict["translation"]
        axis_angle_rotation = cylinder_obstacle_dict["rotation"]
        height = cylinder_obstacle_dict["height"]
        radius = cylinder_obstacle_dict["radius"]
        bottom = cylinder_obstacle_dict["bottom"]
        top = cylinder_obstacle_dict["top"]
        side = cylinder_obstacle_dict["side"]
        subdivision = cylinder_obstacle_dict["subdivision"]

        request = SetCylinderParameters.Request()
        request.cylinder_def = cylinder_def
        request.translation.x = float(translation[0])
        request.translation.y = float(translation[1])
        request.translation.z = float(translation[2])
        request.rotation.x = float(axis_angle_rotation[0])
        request.rotation.y = float(axis_angle_rotation[1])
        request.rotation.z = float(axis_angle_rotation[2])
        request.rotation.angle = float(axis_angle_rotation[3])
        request.height = float(height)
        request.radius = float(radius)
        request.bottom = bool(bottom)
        request.top = bool(top)
        request.side = bool(side)
        request.subdivision = int(subdivision)
        future = self.cylinder_params_cli.call_async(request)
        rclpy.spin_until_future_complete(self.__node, future)
        return future.result()

    def set_sphere_parameters(self, sphere_obstacle_dict):
        sphere_def = sphere_obstacle_dict["def"]
        translation = sphere_obstacle_dict["translation"]
        radius = sphere_obstacle_dict["radius"]
        subdivision = sphere_obstacle_dict["subdivision"]
        ico = sphere_obstacle_dict["ico"]

        request = SetSphereParameters.Request()
        request.sphere_def = sphere_def
        request.translation.x = float(translation[0])
        request.translation.y = float(translation[1])
        request.translation.z = float(translation[2])
        request.radius = float(radius)
        request.subdivision = int(subdivision)
        request.ico = bool(ico)
    
        future = self.sphere_params_cli.call_async(request)
        rclpy.spin_until_future_complete(self.__node, future)
        return future.result()

    def set_goal_cylinder_parameters(self):
        goal_cylinder_dict = self.get_cylinder_dict("goal_cylinder", translation=[self.goal_point[0], self.goal_point[1], 0], radius=0.3, height=0.1, bottom=False, top=True, side=False)
        self.set_cylinder_parameters(goal_cylinder_dict)  

    def random_obstacle_translation(self):
        min_boundary = -np.array(self.boundary_shape)/2
        max_boundary = np.array(self.boundary_shape)/2
        # x at least 2 meters away from the drone initial position
        x_trans = np.random.uniform(low=2, high=2*max_boundary[0])
        y_trans = np.random.uniform(low=min_boundary[1], high=max_boundary[1])
        # z
        z_trans = np.random.normal(loc=2, scale=0.1)
        return [x_trans, y_trans, z_trans]

    def random_obstacle_rotation(self):
        """
        Generate a random axis-angle rotation for an obstacle.

        Returns:
            list: Random axis-angle rotation [x, y, z, angle].
        """
        axis = np.random.normal(size=3)
        axis = axis / np.linalg.norm(axis)  # Normalize the axis
        angle = np.random.uniform(0, 2 * np.pi)
        return axis.tolist() + [angle]

    def randomize_obstacle(self, obstacle_dict):
        if obstacle_dict["type"] == "box":
            return self.randomize_box(obstacle_dict)
        elif obstacle_dict["type"] == "cylinder":
            return self.randomize_cylinder(obstacle_dict)
        elif obstacle_dict["type"] == "sphere":
            return self.randomize_sphere(obstacle_dict)
        else:
            self.__node.get_logger().error(f"Obstacle type {obstacle_dict['type']} not recognized.")
            return None
        
    def randomize_box(self, box_dict):
        box_dict["translation"] = self.random_obstacle_translation()
        box_dict["rotation"] = self.random_obstacle_rotation()
        if box_dict["translation"][0] < 3 and np.absolute(box_dict["translation"][1]) < 2:
            high_size = 0.5
        else:
            high_size = 1
        box_dict["size"] = np.random.uniform(low=0.2, high=high_size, size=(3,)).tolist()
        return box_dict
    
    def randomize_cylinder(self, cylinder_dict):
        cylinder_dict["translation"] = self.random_obstacle_translation()
        cylinder_dict["rotation"] = self.random_obstacle_rotation()
        if cylinder_dict["translation"][0] < 3 and np.absolute(cylinder_dict["translation"][1]) < 2:
            high_size = 0.5
        else:
            high_size = 1
        cylinder_dict["radius"] = np.random.uniform(low=0.2, high=high_size)
        cylinder_dict["height"] = np.random.uniform(low=0.2, high=high_size)
        return cylinder_dict
    
    def randomize_sphere(self, sphere_dict):
        sphere_dict["translation"] = self.random_obstacle_translation()
        sphere_dict["rotation"] = self.random_obstacle_rotation()
        if sphere_dict["translation"][0] < 3 and np.absolute(sphere_dict["translation"][1]) < 2:
            high_radius = 0.5
        else:
            high_radius = 1
        sphere_dict["radius"] = np.random.uniform(low=0.2, high=high_radius)
        return sphere_dict

    def init_obstacles_from_dict(self):
        pass

    def get_box_dict(self, box_def, translation=[0, 0, 0], rotation=[0, 0, 1, 0], size=[1,1,1]):
        return {"type": "box", "def": box_def, "translation": translation, "rotation": rotation, "size": size}

    def get_cylinder_dict(self, cylinder_def, translation=[0, 0, 0], rotation=[0, 0, 1, 0], height=1, radius=1, bottom=True, top=True, side=True, subdivision=16):
        return {"type": "cylinder", "def": cylinder_def, "translation": translation, "rotation": rotation, "height": height, "radius": radius, "bottom": bottom, "top": top, "side": side, "subdivision": subdivision}

    def get_sphere_dict(self, sphere_def, translation=[0, 0, 0], radius=1, subdivision=1, ico=True):
        return {"type": "sphere", "def": sphere_def, "translation": translation, "radius": radius, "subdivision": subdivision, "ico": ico}

    def generate_simple_obstacles(self, obstacles_number=1):
        """
        Initialize simple obstacles. One box, one sphere and one cylinder.
        """
        for i in range(1, obstacles_number+1):
            self.obstacles.append(self.get_box_dict("obstacle_box_"+str(i)))
            self.obstacles.append(self.get_sphere_dict("obstacle_sphere_"+str(i)))
            self.obstacles.append(self.get_cylinder_dict("obstacle_cylinder_"+str(i)))
        self.obstacles.append(self.get_box_dict("obstacle_box_2"))
        self.obstacles.append(self.get_sphere_dict("obstacle_sphere_2"))
        self.obstacles.append(self.get_cylinder_dict("obstacle_cylinder_2"))
    
    def randomize_goal_point(self, change_propability=1):
        if np.random.rand() < change_propability:
            max_boundary = np.array(self.boundary_shape)/2
            # self.goal_point = np.random.uniform(low=max_boundary+1, high=max_boundary+4, size=(2,))*np.append(np.array([1]), np.random.choice([-1, 1], size=(1,)))
            x_goal = np.random.uniform(low=2*max_boundary[0]+1.5, high=2*max_boundary[0]+4)
            y_goal = np.random.uniform(low=-max_boundary[1], high=max_boundary[1])
            self.goal_point = np.array([x_goal, y_goal])
            self.set_goal_cylinder_parameters()
            # self.__node.get_logger().info(f"New goal point: {self.goal_point}")
        return self.goal_point

    def randomize_enviroment(self, change_propability=1):
        for obstacle in self.obstacles:
            if np.random.rand() < change_propability:
                self.set_obstacle_parameters(self.randomize_obstacle(obstacle))
    
    def __del__(self):
        if self.__node is not None:
            self.__node.destroy_node()
            print("rand Node destroyed")