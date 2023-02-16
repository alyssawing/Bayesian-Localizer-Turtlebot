#!/usr/bin/env python
import rospy
import math
from geometry_msgs.msg import Twist
from std_msgs.msg import String, Float64MultiArray
import numpy as np
import colorsys


class BayesLoc:
    def __init__(self, p0, colour_codes, colour_map):
        self.colour_sub = rospy.Subscriber(
            "mean_img_rgb", Float64MultiArray, self.colour_callback
        )
        self.line_sub = rospy.Subscriber("line_idx", String, self.line_callback)
        self.cmd_pub = rospy.Publisher("cmd_vel", Twist, queue_size=1)

        self.num_states = len(p0)
        self.colour_codes = colour_codes    # reference rgb values
        self.colour_map = colour_map        # list of office colours
        self.probability = p0
        self.state_prediction = np.zeros(self.num_states)

        self.cur_colour = None  # most recent measured colour

    def colour_callback(self, msg):
        """
        callback function that receives the most recent colour measurement from the camera.
        """
        self.cur_colour = np.array(msg.data)  # [r, g, b]
        print(self.cur_colour)

    def line_callback(self, msg):
        """
        TODO: Complete this with your line callback function from lab 3.
        """
        self.line_index = msg.data

    def wait_for_colour(self):
        """Loop until a colour is received."""
        rate = rospy.Rate(100)
        while not rospy.is_shutdown() and self.cur_colour is None:
            rate.sleep()

    def state_model(self, u):
        """
        State model: p(x_{k+1} | x_k, u)

        TODO: complete this function
        """

        # state model p(x_k+1 | x_k = X, u_k)
        p_state =  [[0.85, 0.10, 0.05],  # u = -1
                    [0.05, 0.90, 0.05],  # u =  0
                    [0.05, 0.10, 0.85]]  # u = +1
                    # X-1    X     X+1
        
        return p_state[u+1]


    def measurement_model(self, x):
        """
        Measurement model p(z_k | x_k = colour) - given the pixel intensity,
        what's the probability that of each possible colour z_k being observed?
        """
        if self.cur_colour is None:
            self.wait_for_colour()

        prob = np.zeros(len(colourCodes))

        """
        TODO: You need to compute the probability of states. You should return a 1x5 np.array
        Hint: find the euclidean distance between the measured RGB values (self.cur_colour)
            and the reference RGB values of each colour (self.ColourCodes).
        """

        # given rgb values
        r, g, b = self.cur_colour
        # and reference rgb values
        # compute distance between reference and each rgb value
        distances = np.zeros(len(colour_codes))
        for i in range(len(colour_codes)):
            distances[i] = ((r-self.colour_codes[i][0])**2 + (g-self.colour_codes[i][1])**2 + (b-self.colour_codes[i][2])**2)**0.5
        # sum all the distances
        total = sum(distances)
        # return each distance divided by the total sum
        prob = distances/total 
        return prob

    def state_predict(self):
        rospy.loginfo("predicting state")
        """
        TODO: Complete the state prediction function: update
        self.state_prediction with the predicted probability of being at each
        state (office)
        """

        p_state = self.state_model(u)   # don't know what u is
        for i in range(len(p0)):
            s = 0
            # x_k+1 is X-1
            s += p_state[u[k]+1][0] * x[k][(i+1)%11] 
            # x_k+1 is X (stays in the same state)
            s += p_state[u[k]+1][1] * x[k][i%11]
            # x_k+1 is X+1 (state is the next one)
            s += p_state[u[k]+1][2] * x[k][(i-1)%11]
            # put into state prediction
            state_prediction[i] = s 

    def state_update(self):
        rospy.loginfo("updating state")
        """
        TODO: Complete the state update function: update self.probabilities
        with the probability of being at each state
        """


if __name__ == "__main__":

    # This is the known map of offices by colour
    # 0: red, 1: green, 2: blue, 3: yellow, 4: line
    # current map starting at cell #2 and ending at cell #12
    colour_map = [3, 0, 1, 2, 2, 0, 1, 2, 3, 0, 1]

    # TODO calibrate these RGB values to recognize when you see a colour
    # NOTE: you may find it easier to compare colour readings using a different
    # colour system, such as HSV (hue, saturation, value). To convert RGB to
    # HSV, use:
    # h, s, v = colorsys.rgb_to_hsv(r / 255.0, g / 255.0, b / 255.0)
    # colour_codes = [
    #     [167, 146, 158],  # red
    #     [163, 184, 100],  # green
    #     [173, 166, 171],  # blue
    #     [167, 170, 117],  # yellow
    #     [150, 150, 150],  # line
    # ]

    colour_codes = [
        [235, 70, 123],  # red
        [145, 180, 162],  # green
        [178, 167, 184],  # blue
        [186.7, 170.6, 150.5],  # yellow
        [175, 158, 162],  # line
    ]

    colour_codes = [
    [203.3, 109.76, 147.2],  # red
    [120, 139, 130],  # green
    [161.5, 152, 172],  # blue
    [186.7, 170.6, 137.5],  # yellow
    [175, 158, 162],  # line
    ]

    colour_codes_hsv = [[0.9332905708787685, 0.4601082144613871, 0.7972549019607844],
                        [0.42105263157894735, 0.13669064748201432, 0.5450980392156862],
                        [0.7458333333333332, 0.11627906976744193, 0.6745098039215687],
                        [0.09254143646408837, 0.19389394750937322, 0.732156862745098],
                        [0.9607843137254903, 0.09714285714285711, 0.6862745098039216]]

    # initial probability of being at a given office is uniform
    p0 = np.ones_like(colour_map) / len(colour_map)

    localizer = BayesLoc(p0, colour_codes, colour_map)

    rospy.init_node("final_project")
    rospy.sleep(0.5)
    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        """
        TODO: complete this main loop by calling functions from BayesLoc, and
        adding your own high level and low level planning + control logic
        """
        rate.sleep()

    rospy.loginfo("finished!")
    rospy.loginfo(localizer.probability)
