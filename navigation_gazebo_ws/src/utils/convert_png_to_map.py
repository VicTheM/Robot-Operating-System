import numpy as np
import cv2
import math
import os.path
 
strSrcFileName = "../maps/floor_plans/maze_01_binary.png"
image = cv2.imread(strSrcFileName)

strDstFileName = "maze_01_binary"
strDstLocation = "../maps/"

# We scale the map, so it is 40 meters wide
img_width_meters = 40

width, height, _ = image.shape

x1 = [0,width,  0,0]
y1 = [0,0,      0,height]

deltax = img_width_meters
dx = math.sqrt((x1[1]-x1[0])**2 + (y1[1]-y1[0])**2)*.05
sx = deltax / dx

deltay = img_width_meters * float(height) / width
dy = math.sqrt((x1[3]-x1[2])**2 + (y1[3]-y1[2])**2)*.05
sy = deltay/dy 

res = cv2.resize(image, None, fx=sx, fy=sy, interpolation = cv2.INTER_CUBIC)

# Convert to grey
res = cv2.cvtColor(res, cv2.COLOR_BGR2GRAY)
#cv2.imwrite("KEC_BuildingCorrected.pgm", res );
#      cv2.imshow("Image2", res)

completeFileNameMap = os.path.join(strDstLocation, strDstFileName +".pgm")
completeFileNameYaml = os.path.join(strDstLocation, strDstFileName +".yaml")
      
cv2.imwrite(completeFileNameMap, res )

yaml = open(completeFileNameYaml, "w")

yaml.write("image: " + strDstFileName + ".pgm\n")
yaml.write("resolution: 0.050000\n")
yaml.write("origin: [" + str(-1) + "," +  str(-1) + ", 0.000000]\n")
yaml.write("negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196")
yaml.close()
