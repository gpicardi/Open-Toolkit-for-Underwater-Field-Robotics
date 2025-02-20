#!/usr/bin/env python3
import xml.etree.ElementTree as ET
import numpy as np
import scipy.optimize as optimize

class LegMath:
    """
    Class that performs mathematical calculations related to leg kinematics.

    Attributes:
        params (dict): Dictionary containing parameters loaded from an XML file.
        lower_limits (list): List of lower limits for joint angles.
        upper_limits (list): List of upper limits for joint angles.
        initial_guess (list): List of initial guesses for joint angles.

    Methods:
        fw_kine(q):
            Performs forward kinematics calculation for a given set of joint angles.

        inv_kine(ee_pos, knee_up):
            Performs inverse kinematics calculation for a given end-effector position and knee configuration.

        inv_kine_num(ee_pos):
            Performs numerical inverse kinematics calculation using optimization to find joint angles that minimize the difference between desired and computed end-effector positions.

        wrap_angle(angle):
            Wraps an angle to the range (-π, π].

    """
    def __init__(self):
        """
        Initializes the LegMath class by loading parameters from an XML file and setting up default values for attributes.
        """
        tree = ET.parse("../description/silver_constants.xacro")
        root = tree.getroot()

        self.params = {}

        self.lower_limits = [-np.pi/2, -np.pi, -np.pi]
        self.upper_limits = [np.pi/2, np.pi, np.pi]
        self.initial_guess = [0.0, 0.0, 0.0]

        # Extract parameters from the XML file and store them in the params dictionary
        for property_elem in root.findall(".//xacro:property", {"xacro": "http://www.ros.org/wiki/xacro"}):
            name = property_elem.attrib["name"]
            value = property_elem.attrib["value"]
            self.params[name] = value      

    def fw_kine(self, q):
        """
        Performs forward kinematics calculation for a given set of joint angles.

        Args:
            q (list): List of joint angles [q1, q2, q3].

        Returns:
            list: List of end-effector position [x, y, z].

        """
        q1 = q[0]
        q2 = q[1]
        q3 = q[2]

        z = float(self.params["femur_joint_Oz"]) + float(self.params["femur_link_dx"])*np.sin(q2 - np.pi/2) + float(self.params["tibia_link_dx"])*np.sin(q2 - np.pi/2 + q3)
        y_c = float(self.params["femur_joint_Oy"]) + float(self.params["femur_link_dx"])*np.cos(q2 - np.pi/2) + float(self.params["tibia_link_dx"])*np.cos(q2 - np.pi/2 + q3)
        x_c = float(self.params["femur_joint_Ox"]) + float(self.params["tibia_joint_Oz"])
        x = x_c*np.cos(q1) - y_c*np.sin(q1)
        y = x_c*np.sin(q1) + y_c*np.cos(q1)

        return np.array([x, y, z])

    def inv_kine(self, ee_pos, knee_up, right_side = 0):
        """
        Performs inverse kinematics calculation for a given end-effector position and knee configuration.

        Args:
            ee_pos (list): List of end-effector position [x, y, z].
            knee_up (bool): Determines the knee configuration. True if knee is up, False if knee is down.

        Returns:
            list: List of joint angles [q1, q2, q3].

        
        ATTENTION: If the leg has y_c < 0 this function does not work. I haven't found a way to 
        discriminate this case yet
        """

        x = ee_pos[0]
        y = ee_pos[1]
        z = ee_pos[2]

        # Determine the sign for knee configuration
        if knee_up:
            knee_sign = 1
        else:
            knee_sign = -1

        r = np.sqrt(x**2 + y**2)
        #print("r: ", r)

        if x == 0 and y == 0:
            q1 = 0.0
        else:
            alpha = np.arctan2(y,x)
            #print("alpha: ", alpha)
            try:
                beta = np.arccos((float(self.params["femur_joint_Ox"]) + float(self.params["tibia_joint_Oz"]))/r)       
            except:
                print("error in calculation of beta")
                return [np.nan, np.nan, np.nan]
            
            #print("beta: ", beta) 
            q1 = alpha - beta

        #print("q1: ", q1)
        y_c = r*np.sin(beta)
        #print("y_c: ", y_c)
        a = y_c - float(self.params["femur_joint_Oy"])
        #print("a: ", a)
        b = z - float(self.params["femur_joint_Oz"])
        #print("b: ", b)
        c = np.sqrt(a**2+b**2)
        #print("c: ", c)
        try:
            # s1 > 0 if knee is up and s1 < 0 if knee is down
            s1 = knee_sign*np.arccos((-float(self.params["tibia_link_dx"])**2 + float(self.params["femur_link_dx"])**2 + c**2)/(2*float(self.params["femur_link_dx"])*c))
            #print("s1: ", s1)
        except:
            print("error in the calculation of s1")
            return [np.nan, np.nan, np.nan]
        try:
            #s3 = np.arccos(a/c)
            s3 = np.arctan2(b,a)
            #print("s3: ", s3)
        except:
            print("error in the calculation of s3")
            return [np.nan, np.nan, np.nan]
        try:            
            s4 = np.arccos((-c**2 + float(self.params["tibia_link_dx"])**2 + float(self.params["femur_link_dx"])**2)/(2*float(self.params["tibia_link_dx"])*float(self.params["femur_link_dx"])))
            #print("s4: ", s4)
        except:
            print("error in the calculation of s4")
            return [np.nan, np.nan, np.nan]
        q2 = np.pi/2 + s3 + s1
        q3 = knee_sign*(s4 - np.pi) #account for knee_sign  
               
        joint_angles = np.array([wrap_angle(q1), wrap_angle(q2), wrap_angle(q3)]) 
        if right_side:
            joint_angles[0] = -joint_angles[0]
        return joint_angles.reshape(3,1) #column array
    
    def inv_kine_num(self, ee_pos):
        # Define the objective function to minimize the difference between the desired and computed end-effector positions
        def objective(q):
            ee_reconstructed = self.fw_kine(q)
            error = [desired - computed for desired, computed in zip(ee_pos, ee_reconstructed)]
            return sum([e**2 for e in error])

        # Define bounds for the joint angles
        bounds = [(lower_limit, upper_limit) for lower_limit, upper_limit in zip(self.lower_limits, self.upper_limits)]

        # Solve for the optimal joint angles that minimize the objective function
        result = optimize.minimize(objective, self.initial_guess, bounds=bounds)
        q_reconstructed = result.x

        return q_reconstructed


def wrap_angle(angle):
    return (angle + np.pi) % (2 * np.pi) - np.pi