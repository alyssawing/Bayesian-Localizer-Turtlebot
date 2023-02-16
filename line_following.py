#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import UInt32
from std_msgs.msg import Float64
import colorsys
from std_msgs.msg import String, Float64MultiArray

class Controller(object):
    line_index = 0
    x_estimate = 0
    rgb = [0, 0, 0]

    colour_codes = [
        [203.3, 109.76, 147.2],  # red
        [120, 139, 130],  # green
        [161.5, 152, 172],  # blue
        [186.7, 170.6, 150.5],  # yellow
        [175, 158, 162],  # line
    ]

    colour_codes_hsv = [
        [0.9332905708787685, 0.4601082144613871, 0.7972549019607844],
        [0.42105263157894735, 0.13669064748201432, 0.5450980392156862],
        [0.76, 0.149, 0.562],
        [0.09254143646408837, 0.19389394750937322, 0.732156862745098],
        [0.993, 0.0926, 0.684]
    ]

    # yellow read: 0.077, 0.183, 0.709
    # red read:    0.953, 0.705, 0.902

    colours = ['r', 'g', 'b', 'y', 'n']

    # convert rgb colour codes to hsv: 
    # for colour in colour_codes:
    #     colour_codes_hsv.append(colorsys.rgb_to_hsv(colour[0]/255.0, colour[1]/255.0, colour[2]/255.0)) 

    # print(colour_codes_hsv)

    def __init__(self):
        # publish motor commands
        self.cmd_pub = rospy.Publisher("cmd_vel", Twist, queue_size=1)

        # subscribe to detected line index
        self.color_sub = rospy.Subscriber(
            "line_idx", UInt32, self.camera_callback, queue_size=1
        )

        # subscribe to estimated state
        self.state_sub = rospy.Subscriber(
            "state", Float64, self.state_callback, queue_size=1
        )

        # subscribe to rgb camera 

        self.rgb_sub = rospy.Subscriber(
                "mean_img_rgb", Float64MultiArray, self.rgb_callback, queue_size=1
        )

    def rgb_callback(self, msg):
        """Callback for line index"""
        # access the value using msg.data
        self.rgb = msg.data

    def camera_callback(self, msg):
        """Callback for line index."""
        # access the value using msg.data
        self.line_index = msg.data

    def state_callback(self, msg):
        """Callback for state"""
        self.x_estimate = msg.data

    

    # define true/false function if at an office or not 
    def is_office(self):
        r, g, b = self.rgb # detected rgb values
        h, s, v = colorsys.rgb_to_hsv(r/255.0, g/255.0, b/255.0)    # detected converting rgb values to hsv values
        h_tolerance = 0.2
        s_tolerance = 0.3
        v_tolerance = 0.14

        # for c in self.colour_codes_hsv[:4]:
        #     if abs(h - c[0]) < h_tolerance and abs(s - c[1]) < s_tolerance and \
        #         abs(v - c[2]) < v_tolerance:
        #         return True

        # return False    # no office (colour block) detected 

        hsv = self.colour_codes_hsv
        for i in range(len(hsv[:-2])):
            if abs(h - hsv[i][0]) < h_tolerance and abs(s - hsv[i][1]) < s_tolerance and \
                abs(v - hsv[i][2]) < v_tolerance:
                return self.colours[i]
        
        return None
    

    def is_line(self):
        r, g, b = self.rgb # detected rgb values
        h, s, v = colorsys.rgb_to_hsv(r/255.0, g/255.0, b/255.0)    # detected converting rgb values to hsv values
        h_tolerance = 0.15
        s_tolerance = 0.05
        v_tolerance = 0.14

        hsv = self.colour_codes_hsv
        if abs(h - hsv[-1][0]) < h_tolerance and abs(s - hsv[-1][1]) < s_tolerance and \
                abs(v - hsv[-1][2]) < v_tolerance:
            return self.colours[-1]

        return None
            

    def bang_bang(self):
        """
        TODO: complete the function to follow the line
        """
        desired = 320
        twist = Twist()
        twist.linear.x=0.1 #moves at 0.2m/s
        rate = rospy.Rate(30)    # 10 Hz
        
        while not rospy.is_shutdown():
            # go until we shut down ros
            actual = self.line_index #actual position 
            error = desired - actual
            if error < 0:
            	twist.angular.z = -0.8 #correction: steer towards the right (clockwise)
            elif error > 0:
            	twist.angular.z = 0.8 #correction: steer towards the left (counterclockwise)
            else:
            	twist.angular.z = 0 #correction: nothing
            self.cmd_pub.publish(twist) #go forwards + do angular twist correction
            rate.sleep()
            
            
    def p(self): #proportional (part 4.2)
        """
        TODO: complete the function to follow the line
        """
        desired = 320
        kp = 0.004 #proportional gain 
        twist = Twist()
        twist.linear.x = 0.15 #moves at 0.1m/s
        rate = rospy.Rate(30)    # 30Hz
        
        while not rospy.is_shutdown():
            # go until we shut down ros
            actual = self.line_index #actual position 
            error = desired - actual
            correction = kp * error
            max_correction = 1.4 #max angular velocity
            
            if correction > max_correction:
            	twist.angular.z = max_correction
            elif correction < -1*max_correction:
            	twist.angular.z = -1*max_correction
            else:
            	twist.angular.z = correction
            
            self.cmd_pub.publish(twist) #go forwards + do angular twist correction
            rate.sleep()
        
    def pi_control(self): #integral (part 4.3)
        """
        TODO: complete the function to follow the line
        """
        desired = 320
        integral = 0
        kp = 0.004 #proportional gain 
        ki = 0.00001 #integral gain
        
        twist = Twist()
        twist.linear.x = 0.15 #moves at 0.1m/s
        rate = rospy.Rate(30)    # 30Hz
        
        while not rospy.is_shutdown():
            # go until we shut down ros
            actual = self.line_index #actual position 
            error = desired - actual
            integral = integral + error
            correction = (kp * error) + (ki * integral)
            max_correction = 1.5 #max angular velocity
            
            #rospy.loginfo(error)
            #rospy.loginfo(integral)
            print("error: ", error)
            print("integral: ", integral)
            
            if correction > max_correction:
            	twist.angular.z = max_correction
            elif correction < -1*max_correction:
            	twist.angular.z = -1*max_correction
            else:
            	twist.angular.z = correction
            	
            	if abs(error) <  50: #very good position
            		integral = 0.7 * integral #reset integral
            
            self.cmd_pub.publish(twist) #go forwards + do angular twist correction
            rate.sleep()

    def pid_control(self): #derivative (part 4.4)
        """
        TODO: complete the function to follow the line
        """
        desired = 320
        integral = 0
        derivative = 0
        lasterror = 0
  
        # test:
        # kp = 0.0047
        # ki = 00.0000001
        # kd = 0.05

        kp = 0.003#47
        ki = 00.0000001
        kd = 0.003#5

        speed = 0.07
        midspeed = speed 
        espeed = speed 
        minspeed = speed

        # tolerance = 0.0005
        tolerance = 0.002


        twist = Twist()
        twist.linear.x = speed #moves at 0.1m/s
        r = 120
        rate = rospy.Rate(r)    # 30Hz
        
        while not rospy.is_shutdown():
            # go until we shut down ros
            actual = self.line_index #actual position 
            r, g, b = self.rgb #rgb values
            h, s, v = colorsys.rgb_to_hsv(r/255.0, g/255.0, b/255.0)    # converting rgb values to hsv values
            error = desired - actual

            if abs(error) > 100: #if error is greater than 150 pixels (?)
                twist.linear.x = minspeed #slows down when encounters sharper turns
            if abs(error) > 75:
                twist.linear.x = espeed
            elif abs(error) > 50:
                twist.linear.x = midspeed
            else:
                twist.linear.x = speed #go faster when more accurate


            integral = integral + error
            derivative = error - lasterror
            
            correction = (kp * error) + (ki * integral) + (kd * derivative)
            # max_correction = 1.82 #max angular velocity
            max_correction = 1.3
                        
            if correction > max_correction:
            	twist.angular.z = max_correction
            elif correction < -1*max_correction:
            	twist.angular.z = -1*max_correction
            else:
            	twist.angular.z = correction
            	
            	if abs(error) <  50: #very good position
            	    integral = 0.7 * integral #reset integral
            
            # if at an office, go straight
            # office = self.is_office()
            # if office or not self.is_line():
            #     twist.angular.z = 0
            #     print("office:\t", office, "\thsv:\t", f"({h}, {s}, {v}")

            # else:
            #     print("at line,\t\thsv:\t", f"({h}, {s}, {v}")
            #     if abs(error) > 300:
            #         twist.angular.z = 0
            
            # if not at a line, go straight
            if not self.is_line():
                twist.angular.z = 0
                print("office:\t", self.is_office(), "\thsv:\t", f"({h}, {s}, {v}")
            else:
                print("at line,\t\thsv:\t", f"({h}, {s}, {v}")
                # if abs(error) > 300:
                #     twist.angular.z = 0

            # print("is_office:", office)
            # print("hsv: ", h, s, v)

            # print("self.x_estimate:", self.x_estimate)

            # labe 3 - stopping at predetermined points
            # if abs(self.x_estimate - 0.61) < tolerance or abs(self.x_estimate - 1.22) < tolerance or\
            #         abs(self.x_estimate - 2.44) < tolerance or abs(self.x_estimate - 3.05) < 0.001:
            #     # old_twist = twist
            #     for i in range(2*r):    # 2 * 120 Hz = 240 cycles
            #         twist.linear.x = 0
            #         twist.angular.z = 0
            #         self.cmd_pub.publish(twist)
            #         print("speed x: ", twist.linear.x)
            #         rate.sleep()
            #     # twist = old_twist

            twist.linear.x = speed
            self.cmd_pub.publish(twist) #go forwards + do angular twist correction
            # print("speed x: ", twist.linear.x)
            lasterror = error
            
            rate.sleep()


if __name__ == "__main__":
    rospy.init_node("lab3")
    controller = Controller()
    controller.pid_control()
