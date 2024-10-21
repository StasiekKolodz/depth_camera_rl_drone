"ROS2 driver for webots supervisor robot responsible for managing webots simulation (eg. reseting)"

import rclpy
import math
from drone_interfaces.srv import SetRobotPose, SetRobotPoseRelative, SetObstacleParameters, SetBoxParameters, SetCylinderParameters, SetSphereParameters
from scipy.spatial.transform import Rotation
import numpy as np
import time
class WebotsSimManager:
    def init(self, webots_node, properties):
        self.__robot = webots_node.robot
        self.__timestep = int(self.__robot.getBasicTimeStep())

        # ROS interface
        rclpy.init(args=None)
        self.__node = rclpy.create_node('webots_sim_manager')
        self.__node.create_service(SetRobotPose, 'set_robot_pose', self.set_robot_pose_callback)
        self.__node.create_service(SetRobotPoseRelative, 'set_robot_pose_relative', self.set_robot_pose_relative_callback)
        self.__node.create_service(SetObstacleParameters, 'set_obstacle_parameters', self.set_obstacle_parameter_callback)
        self.__node.create_service(SetBoxParameters, 'set_box_parameters', self.set_box_parameters_callback)
        self.__node.create_service(SetCylinderParameters, 'set_cylinder_parameters', self.set_cylinder_parameters_callback)
        self.__node.create_service(SetSphereParameters, 'set_sphere_parameters', self.set_sphere_parameters_callback)
        self.__node.get_logger().info(f"sim manager node created")

    def set_robot_pose_callback(self, request, response):
        self.__node.get_logger().info(f"Service request recieved, robot def: {request.robot_def}")
        robot_from_def = self.__robot.getFromDef(request.robot_def)
        translation_field = robot_from_def.getField('translation')
        rotation_field = robot_from_def.getField('rotation')
        
        translation_vec = [request.translation.x, request.translation.y, request.translation.z]
        rotation_axis_angle = [request.rotation.x, request.rotation.y, request.rotation.z, request.rotation.angle]
        self.__node.get_logger().info(f"translation: {translation_vec} rotation: {rotation_axis_angle}")

        translation_field.setSFVec3f(translation_vec)
        rotation_field.setSFRotation(rotation_axis_angle)

        if request.reset_physics:
            robot_from_def.resetPhysics()

        response.result = 1
        return response
    
    def set_robot_pose_relative_callback(self, request, response):
        self.__node.get_logger().info(f"Service request recieved, robot def: {request.robot_def}")
        robot_from_def = self.__robot.getFromDef(request.robot_def)
        translation_field = robot_from_def.getField('translation')
        rotation_field = robot_from_def.getField('rotation')
        
        position = robot_from_def.getPosition()
        orientation = np.array(robot_from_def.getOrientation()).reshape(3,3)

        request_translation_vec = [request.translation.x, request.translation.y, request.translation.z]
        request_axis_angle = [request.rotation.x, request.rotation.y, request.rotation.z, request.rotation.angle]
        
        # Computing absolute rotation by multiplying rotation matrices
        R_relative = Rotation.from_rotvec([i*request_axis_angle[3] for i in request_axis_angle[0:3]])
        R_relative = R_relative.as_matrix()

        R_abs = np.matmul(orientation, R_relative)
        
        R_abs = Rotation.from_matrix(R_abs)
        rot_vec = R_abs.as_rotvec()
        
        angle = np.linalg.norm(rot_vec)
    
        # Handle 0 rotation (rotvec = [0,0,0])
        if angle != 0:
            rot_axis = rot_vec/angle
        else:
            rot_axis = np.array([0,0,1])
        self.__node.get_logger().info(f"angle: {angle}")
        rotation_axis_angle = np.append(rot_axis, angle)

        # Translation: p' R*p + T
        translation_abs = position + np.matmul(orientation, np.array(request_translation_vec))

        # self.__node.get_logger().info(f"translation: {translation_abs} rotation: {rotation_axis_angle}")

        translation_field.setSFVec3f(translation_abs.tolist())
        rotation_field.setSFRotation(rotation_axis_angle.tolist())

        if request.reset_physics:
            robot_from_def.resetPhysics()

        response.result = 1
        return response

    def set_obstacle_parameter_callback(self, request, response):
        self.__node.get_logger().info(f"Set obstacle param srv request recieved, obstacle def: {request.obstacle_def}")
        obstacle_from_def = self.__robot.getFromDef(f'{request.obstacle_def}.obstacle_box_1_boundingObject')
        # bo = obstacle_from_def.getField(f'{request.obstacle_def}.obstacle_box_1_boundingObject')
        sz = obstacle_from_def.getField('size')
        sz.setSFVec3f([1., 1., 1.])
        response.result = 1
        return response

    def set_box_parameters_callback(self, request, response):
        box_def = request.box_def
        self.__node.get_logger().info(f"Set box param srv request received, box def: {box_def}")
        
        # Retrieve the box object
        box_from_def = self.__robot.getFromDef(box_def)
        bo_from_def = self.__robot.getFromDef(f"{box_def}.{box_def}_boundingObject")
        shape_from_def = self.__robot.getFromDef(f"{box_def}.{box_def}_shape.{box_def}_geometry")

        # Set translation
        translation = [request.translation.x, request.translation.y, request.translation.z]
        box_from_def.getField('translation').setSFVec3f(translation)

        # Set rotation
        rotation = [request.rotation.x, request.rotation.y, request.rotation.z, request.rotation.angle]
        box_from_def.getField('rotation').setSFRotation(rotation)

        # Set size
        size = [request.size.x, request.size.y, request.size.z]
        bo_from_def.getField('size').setSFVec3f(size)
        shape_from_def.getField('size').setSFVec3f(size)

        response.result = 1
        return response

    def set_cylinder_parameters_callback(self, request, response):
        cylinder_def = request.cylinder_def
        self.__node.get_logger().info(f"Set cylinder param srv request received, cylinder def: {cylinder_def}")
        
        
        self.__node.get_logger().info(f"{cylinder_def}.{cylinder_def}_shape.{cylinder_def}_geometry")
        
        # Retrieve object from def
        cylinder_from_def = self.__robot.getFromDef(cylinder_def)
        shape_from_def = self.__robot.getFromDef(f"{cylinder_def}.{cylinder_def}_shape.{cylinder_def}_geometry")
        bo_from_def = self.__robot.getFromDef(f"{cylinder_def}.{cylinder_def}_boundingObject")

        # Set translation
        translation = [request.translation.x, request.translation.y, request.translation.z]
        cylinder_from_def.getField('translation').setSFVec3f(translation)

        # Set rotation
        rotation = [request.rotation.x, request.rotation.y, request.rotation.z, request.rotation.angle]
        cylinder_from_def.getField('rotation').setSFRotation(rotation)

        # Set bottom
        bottom = request.bottom
        shape_from_def.getField('bottom').setSFBool(bottom)
        bo_from_def.getField('bottom').setSFBool(bottom)

        # Set height
        height = request.height
        bo_from_def.getField('height').setSFFloat(height)
        shape_from_def.getField('height').setSFFloat(height)

        # Set radius
        radius = request.radius
        bo_from_def.getField('radius').setSFFloat(radius)
        shape_from_def.getField('radius').setSFFloat(radius)

        # Set top
        top = request.top
        bo_from_def.getField('top').setSFBool(top)
        shape_from_def.getField('top').setSFBool(top)

        # Set side
        side = request.side
        bo_from_def.getField('side').setSFBool(side)
        shape_from_def.getField('side').setSFBool(side)

        # Set subdivision
        subdivision = request.subdivision
        bo_from_def.getField('subdivision').setSFInt32(subdivision)
        shape_from_def.getField('subdivision').setSFInt32(subdivision)
        
        response.result = 1
    
        return response

    def set_sphere_parameters_callback(self, request, response):
        sphere_def = request.sphere_def
        self.__node.get_logger().info(f"Set sphere param srv request received, sphere def: {sphere_def}")
        
        # Retrieve the sphere object
        sphere_from_def = self.__robot.getFromDef(sphere_def)
        bo_from_def = self.__robot.getFromDef(f"{sphere_def}.{sphere_def}_boundingObject")
        shape_from_def = self.__robot.getFromDef(f"{sphere_def}.{sphere_def}_shape.{sphere_def}_geometry")

        # Set translation
        translation = [request.translation.x, request.translation.y, request.translation.z]
        sphere_from_def.getField('translation').setSFVec3f(translation)

        # Set radius
        radius = request.radius
        bo_from_def.getField('radius').setSFFloat(radius)
        shape_from_def.getField('radius').setSFFloat(radius)

        # Set subdivision
        subdivision = request.subdivision
        bo_from_def.getField('subdivision').setSFInt32(subdivision)
        shape_from_def.getField('subdivision').setSFInt32(subdivision)

        # Set ico
        ico = request.ico
        bo_from_def.getField('ico').setSFBool(ico)
        shape_from_def.getField('ico').setSFBool(ico)

        response.result = 1
        return response

    def step(self):
        rclpy.spin_once(self.__node, timeout_sec=0)

