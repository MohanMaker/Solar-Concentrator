import math
import csv

#define constants
dimension = 5 # number of mirrors in the x and y axis (2 x 2, 5x5, 9 x 9, etc.)
if dimension % 2 == 0:
    middle = dimension * .5 + .5
else: 
    middle = round(dimension * .5,0)
print str(middle)
focal_height = 1355
focal_depth = 304.8
mirror_height = 100 #mount + mirror + pegboard
mount_dimension = 20
adj_focal_height = focal_height - mirror_height
mirror_dimension = 63.5 #mirror size (2.5")
mirror_spacing = 12.7 # (0.5")
helio =[]

csv = open('helio.csv', 'w')
csv.write("Angle Calculation Solar Tracker Spreadsheet")
csv.write("\nrow-column,h1,h2,h3,h4")

row = 1
while row <= dimension:
    column = 1
    depth1 = focal_depth + (row - 1 + .5) * mirror_dimension + (row-1) * mirror_spacing
    print "row: " + str(row) + " depth1: " + str(depth1)
    fb_angle1 = math.degrees(math.atan(adj_focal_height/depth1))
    fb_angle2 = 90 - fb_angle1
    fb_angle = fb_angle1 + fb_angle2 / 2
    fb_angle = 90 + fb_angle
    print("mohan fb_angle: " + str(fb_angle))
    #fb_angle = (90 - math.degrees(math.atan(adj_focal_height/depth1))) * .5 
    #print("rachael fb_angle: " + str(fb_angle))
    #print str(row), str(column) + " fb_angle: " +  str(fb_angle)
    #print "fb_angle: " + str(fb_angle)   
    while column <= dimension:
        if column - middle > 0:
            side = "right"
        else:
            side = "left"  
        print str(side)
        if side == "right":
            width1 = (column - middle) * mirror_dimension + (column - middle) * mirror_spacing
        else:
            width1 = (column - middle) * mirror_dimension * -1 - (column - middle) * mirror_spacing
        #width1 = (column - middle) * mirror_dimension + (column - middle) * mirror_spacing
        #print "column: " + str(column) + " width1: " + str(width1)
        if width1 != 0:
            #if side == "right":
                #base1 = math.sqrt((width1 * width1) + (depth1 * depth1))
                rl_angle1 = math.degrees(math.atan(adj_focal_height/width1))
                rl_angle2 = 90 - rl_angle1
                rl_angle = rl_angle1 + rl_angle2 / 2
                rl_angle = rl_angle + 90
                rl_angle = 180 - rl_angle
            # if side == "left":
            #     rl_angle1 = math.degrees(math.atan(adj_focal_height/width1))
            #     rl_angle2 = 90 - rl_angle1
            #     rl_angle = rl_angle1 + rl_angle2 / 2
            #     rl_angle = rl_angle + 90     
        else:
            rl_angle = 0
        print str(row), str(column) + " rl_angle: " + str(rl_angle)    
        #print "row: " + str(row) + " column: " + str(column) + " fb_angle: " + str(fb_angle) + " rl_angle: " + str(rl_angle)
        # get heights from angles
        if side == "right":
            h1 = mount_dimension * math.tan(math.radians(180 - fb_angle))
            h2 = 0 
            h3 = mount_dimension * math.tan(math.radians(rl_angle))
            h4 = h1 + h3
            center_height = (max(h1,h2,h3,h4) - min(h1,h2,h3,h4)) * .5 #get center height for adjustment
        if side == "left":
            # h1 = mount_dimension * math.tan(math.radians(180 - fb_angle))
            # h2 = 0 
            # h3 = mount_dimension * math.tan(math.radians(rl_angle))
            # h4 = h1 + h3
            h4 = mount_dimension * math.tan(math.radians(180 - fb_angle))
            h3 = 0
            h2 = mount_dimension * math.tan(math.radians(rl_angle))
            h1 = h2 + h4
            center_height = (max(h1,h2,h3,h4) - min(h1,h2,h3,h4)) * .5 #get center height for adjustment
        helio.append([row,column,fb_angle,rl_angle,h1,h2,h3,h4,center_height])
        #print "row: " + str(row) + " column: " + str(column) + " fb_angle: " + str(fb_angle) + " rl_angle: " + str(rl_angle) + " h1: " + str(h1) + " h2: " + str(h2) + " h3: " + str(h3) + " h4: " + str(h4) + " center_height: " + str(center_height)
        #csv.write("\n" + str(row) + "," + str(column) + "," + str(fb_angle) + "," + str(rl_angle) + "," + str(h1) + "," + str(h2) + "," + str(h3) + "," + str(h4))
        column += 1
    row += 1

#find the max center height
i = 0
max_center_height = helio[i][8]
#print str(helio)
while i < len(helio):
    max_center_height = max(max_center_height,helio[i][8])
    i += 1
print str(max_center_height)

#adjust center heights and write to file
j = 0
while j < len(helio):
    print "max center height: " + str(max_center_height)
    print "this helio's center height: " + str(helio[j][8])
    if(helio[j][8] == max_center_height): #if we're dealing with a mount which already has max_center_height
        #print "I'm finding that helio's max is the max"   
        #print "row: " + str(helio[j][0]) + " column: " + str(helio[j][1]) + " fb_angle: " + str(helio[j][2]) + " rl_angle: " + str(helio[j][3]) + " h1: " + str(helio[j][4]) + " h2: " + str(helio[j][5]) + " h3: " + str(helio[j][6]) + " h4: " + str(helio[j][7]) + " center_height: " + str(helio[j][8])
        csv.write("\n" + str(helio[j][0]) + "-" + str(helio[j][1]) + "," + str(helio[j][4]) + "," + str(helio[j][5]) + "," + str(helio[j][6]) + "," + str(helio[j][7]))
        #csv.write("\n" + str(row) + "," + str(column) + "," + str(fb_angle) + "," + str(rl_angle) + "," + str(h1) + "," + str(h2) + "," + str(h3) + "," + str(h4))
    else: 
        adjust_height = max_center_height - helio[j][8]
        helio[j][4] += adjust_height
        helio[j][5] += adjust_height
        helio[j][6] += adjust_height
        helio[j][7] += adjust_height   
        #print "row: " + str(helio[j][0]) + " column: " + str(helio[j][1]) + " fb_angle: " + str(helio[j][2]) + " rl_angle: " + str(helio[j][3]) + " h1: " + str(helio[j][4]) + " h2: " + str(helio[j][5]) + " h3: " + str(helio[j][6]) + " h4: " + str(helio[j][7]) + " center_height: " + str(helio[j][8])
        csv.write("\n" + str(helio[j][0]) + "-" + str(helio[j][1]) + "," + str(helio[j][4]) + "," + str(helio[j][5]) + "," + str(helio[j][6]) + "," + str(helio[j][7]))
    j += 1
csv.close

# Data is exported into a csv. Can use Parameter I/O extension to import the values into the Fusion 360 parametric model