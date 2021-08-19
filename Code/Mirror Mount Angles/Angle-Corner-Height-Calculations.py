# Angle Mount Calculation Code for Solar Concentrator
    # See https://docs.google.com/spreadsheets/d/1m_uL-1rF_R8tuWbk3YL2R-5y6YyMJneRw9KkN6L4wuk/edit?usp=sharing

# import libraries
import math
import sys
import csv

# define constants
dimension = 10 # array dimensions/number of mirrors in the x and y axis (eg. 2 for 2 x 2, 5 for 5 x 5, 9 for 9 x 9)
focal_height = 1355 # in mm
focal_depth = 304.8 # in mm
mirror_dimension = 63.5 # size of square mirror, in mm
mirror_spacing = 12.7 # spacing between mirrors, in mm
mount_dimension = 20 # length of top part of mirror mount, in mm

# function to calculate angles
def calcAngles(row, column):
    # front-back angle
    depth = focal_depth + (row - 1) * mirror_dimension + 0.5 * mirror_dimension + (row - 1) * mirror_spacing
    print("row " + str(row) + " depth: " + str(depth))
    fb_angle1 = math.degrees(math.atan(focal_height/depth))
    fb_angle2 = 90 - fb_angle1
    fb_angle = fb_angle1 + (fb_angle2 / 2)
    fb_angle = 90 - fb_angle
    print("row " + str(row) + " fb angle: " + str(fb_angle))
    
    # right-left angle
    if dimension % 2 == 0:
        middle = dimension / 2 + 0.5
    else: 
        middle = math.ceil(dimension / 2)
    width = mirror_dimension * math.fabs(middle - column) + mirror_spacing * math.fabs(middle - column)
    print("column " + str(column) + " width: " + str(width))
    if width != 0:
        rl_angle1 = math.degrees(math.atan(focal_height/width))
        rl_angle2 = 90 - rl_angle1
        rl_angle = rl_angle1 + (rl_angle2 / 2)
        rl_angle = 90 - rl_angle
    else:
        rl_angle = 0
    if column - middle > 0:
        side = "right"
    else:
        side = "left" # (or side = "center")
    print("column " + str(column) + " rl angle: " + str(rl_angle) + " (on the " + side + ")")

    # return calculated values
    return fb_angle, rl_angle, side

# function to calculate corner heights (distance from *top* of block)
def cornerHeight(fb_angle, rl_angle, side):
        if side == "right":
            h1 = mount_dimension * math.tan(math.radians(fb_angle))
            h2 = 0 
            h3 = mount_dimension * math.tan(math.radians(rl_angle))
            h4 = h1 + h3
        else: # side == "left" | "center"
            h4 = mount_dimension * math.tan(math.radians(fb_angle))
            h3 = 0
            h2 = mount_dimension * math.tan(math.radians(rl_angle))
            h1 = h2 + h4
        print("corner height: " + str(h1) + ", " + str(h2) + ", " + str(h3) + ", " + str(h4))

        # return calculated values
        return [h1, h2, h3, h4]

# function to adjust center heights
def adjustCenterHeight(helio):
    # find the greatest center height
    max_center_height = -1 * sys.maxsize - 1
    for r in range(len(helio)):
        for c in range(len(helio[r])):
            center_height = (max(helio[r][c]) - min(helio[r][c])) / 2 # get center height for each block
            if center_height > max_center_height:
                max_center_height = center_height
    print("max center height: " + str(max_center_height))

    # adjust the center heights based on the max
    for r in range(len(helio)):
        for c in range(len(helio[r])):
            center_height = (max(helio[r][c]) - min(helio[r][c])) / 2 # find center height for each block
            adjust_height = max_center_height - center_height
            for i in range(4):
                helio[r][c][i] += adjust_height
        
# the code/calling all of the functions
# create helio 2D list/array and call calcAngles and cornerHeight to fill it with h1, h2, h3, h4
helio = [[0 for i in range(dimension)] for j in range(dimension)]
for r in range(len(helio)):
    for c in range(len(helio[r])):
        helio[r][c] = cornerHeight(*calcAngles(r + 1, c + 1))

# adjust center heights
adjustCenterHeight(helio)

# create csv file and write to it
with open('/Users/mohanhathi/Desktop/Projects/Solar-Concentrator/heliostat-angle-and-corner-height-results.csv', 'w', newline='') as f: 
    writer = csv.writer(f) 
    header = ['row-column', 'h1', 'h2', 'h3', 'h4'] 
    writer.writerow(header)
    for r in range(len(helio)):
        for c in range(len(helio[r])):
            data = [(str(r + 1) + "_" + str(c + 1)), helio[r][c][0], helio[r][c][1], helio[r][c][2], helio[r][c][3]]
            writer.writerow(data)

# data is exported into a csv -- then use the CornerHeight code in Fusion 360 Scripts/Extensions to import the values into the fusion 360 parametric model and create the stl files