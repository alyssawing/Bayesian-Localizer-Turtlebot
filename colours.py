import colorsys
import numpy as np

colour_codes = [
    [203.3, 109.76, 147.2],  # red
    [120, 139, 130],  # green
    [161.5, 152, 172],  # blue
    [186.7, 170.6, 150.5],  # yellow
    [175, 158, 162],  # line
]

# colour_codes_hsv = [
# (0.953, 0.667, 0.818),
# (0.317, 0.203, 0.582),
# (0.649, 0.258, 0.775),
# (0.085, 0.134, 0.716),
# (0.960, 0.077, 0.681)
# ]
colour_codes_hsv = []

# convert rgb colour codes to hsv: 
for colour in colour_codes:
    colour_codes_hsv.append(colorsys.rgb_to_hsv(colour[0]/255.0, colour[1]/255.0, colour[2]/255.0)) 


# print(colour_codes_hsv)
for colour in colour_codes_hsv:
    print(colour)


#  [[0.9332905708787685, 0.4601082144613871, 0.7972549019607844],
# [0.42105263157894735, 0.13669064748201432, 0.5450980392156862],
# [0.7458333333333332, 0.11627906976744193, 0.6745098039215687],
# [0.11212737127371275, 0.2635243706480985, 0.732156862745098],
# [0.9607843137254903, 0.09714285714285711, 0.6862745098039216]]


# given rgb values
r, g, b = [100, 200, 130]
# and reference rgb values
# compute distance between reference and each rgb value
distances = np.zeros(len(colour_codes))
for i in range(len(colour_codes)):
    distances[i] = ((r-colour_codes[i][0])**2 + (g-colour_codes[i][1])**2 + (b-colour_codes[i][2])**2)**0.5
# sum all the distances
total = sum(distances)
# return each distance divided by the total sum
prob = distances/total 
print(prob)
print(type(prob))
print(sum(prob))