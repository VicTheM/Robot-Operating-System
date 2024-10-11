# (c) robotics.snowcron.com
# Use: MIT license

import math
import cv2
import numpy as np
from PIL import Image
import os
from pathlib import Path

class AStar():

    def __init__(self):
        pass
    
    def setMap(self, map):
        self.map = map
        self.cost_map = 255.0 / np.maximum(self.map, 1)
        self.height, self.width = map.shape

    def heuristic(self, a, b):
        return math.sqrt((b[0] - a[0])**2 + (b[1] - a[1])**2)
    
    def get_neighbors(self, node, treshold):
        r, c = node
        directions = [(r-1, c), (r+1, c), (r, c-1), (r, c+1)]
        neighbors = []
        for r, c in directions:
            if 0 <= r < self.height and 0 <= c < self.width and self.map[r,c] > treshold:
                neighbors.append((r, c))
        return neighbors
        
    def get_astar_grid_path(self, start, goal, treshold):
        
        openset = set()
        closedset = set()
        openset.add(start)
        
        came_from = {}
        
        g_score = {start: 0}
        f_score = {start: self.heuristic(start, goal)}
        
        while openset:
            
            current = min(openset, key=lambda x: f_score[x])
            
            if current == goal:
                path = []
                while current in came_from:
                    path.append(current)
                    current = came_from[current]
                    
                return path[::-1]
            
            openset.remove(current)
            closedset.add(current)
            
            for neighbor in self.get_neighbors(current, treshold):
                tentative_g_score = g_score[current] + self.cost_map[neighbor[0],neighbor[1]]
                if neighbor in closedset and tentative_g_score >= g_score[neighbor]:
                    continue
                    
                if neighbor not in openset or tentative_g_score < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g_score
                    f_score[neighbor] = g_score[neighbor] + self.heuristic(neighbor, goal)
                    
                    if neighbor not in openset:
                        openset.add(neighbor)
                        
        return None

    # ---

    def loadKeepoutGrid(self, strYamlImageFilePath, nGridSizeX, nGridSizeY):
        
        # TBD: handle return if can not open file 
        with open(strYamlImageFilePath, mode="r", encoding="utf-8") as input_file:
                
            text = input_file.read()
            arrLines = text.splitlines()
            strMapFileName = strYamlImageFilePath[:-4] + "pgm"
            strMapDir = directory = os.path.dirname(strYamlImageFilePath)

            dOriginMetersX = 0
            dOriginMetersY = 0
            for str in arrLines:
                key, value = str.split(":")
                value = value.strip()
                if(key == "mode"):
                    strMode = value
                elif(key == "resolution"):
                    dResolution = float(value)
                elif(key == "origin"):
                    value = value.replace("[", "")
                    value = value.replace("]", "")
                    arrOrigin = value.split(", ")
                    
                    # Note: if "origin" not provided, default is (0,0)
                    dOriginMetersX = float(arrOrigin[0].strip())
                    dOriginMetersY = float(arrOrigin[1].strip())
                    
                elif(key == "negate"):
                    nNegate = int(value)
                elif(key == "occupied_thresh"):
                    nOccupiedThresh = float(value)
                elif(key == "free_thresh"):
                    nFreeThresh = float(value)    
                elif(key == "image"):
                    strMapFileName = os.path.join(strMapDir, value)
                    #print(">>>", strMapFileName)
            
            img = Image.open(strMapFileName)
            
            # ---

            # Coordinates of corners of the map we just loaded in meters
            # Note: origin is left-bottom
            dLeftMeters = dOriginMetersX
            dTopMeters = dOriginMetersY + img.size[1] * dResolution
            dRightMeters = dOriginMetersX + img.size[0] * dResolution
            dBottomMeters = dOriginMetersY

            dGridCellSizeX = img.size[0] * dResolution / nGridSizeX
            dGridCellSizeY = img.size[1] * dResolution / nGridSizeY

            # ---

            img_array = np.array(img).astype(np.uint8)
            #print(">>>", np.amin(img_array), ", ", np.amax(img_array))
            img_array = img_array - 127

            arrGrid = 255 * np.ones((nGridSizeX, nGridSizeY), dtype=np.uint8)
            for i in range(nGridSizeY):
                for j in range(nGridSizeX):
                    cropped = img_array[i*10:(i+1)*10, j*10:(j+1)*10]
                    arrGrid[i, j] = np.amax(cropped)

                    # print("\tmap[", i, ",", j, "] = ", arrGrid[i, j])

            return {'grid': arrGrid, 'img': img, 'originMetersX': dOriginMetersX, 'originMetersY': dOriginMetersY, 
                'gridCellSizeX': dGridCellSizeX, 'gridCellSizeY': dGridCellSizeY, 
                'mode': strMode, 'resolution': dResolution, 'nNegate': nNegate, 
                'nOccupiedThresh': nOccupiedThresh, 'nFreeThresh': nFreeThresh,
            }
        
        return None

    # ---

if __name__ == "__main__":

    # map = 255*np.ones((20,20), dtype=np.uint8)  

    # # Add walls and obstacles
    # map[2:5, 3] = 32

    # map[8, 0:14] = 64
    # map[8, 17:20] = 128

    # map[9, 0:15] = 64
    # map[9, 16:20] = 128

    # map[9, 0:20] = 128

    # map[16:20, 16] = 0

    # Note, that it is row, column, not the other way around
    # start = (0, 0)
    # goal = (19, 10)

    astar = AStar()

    map = astar.loadKeepoutGrid("../maps/keepout_world_01.yaml", 64, 64)["grid"]

    start = (17, 55)
    goal = (14, 59)
    
    astar.setMap(map)
    path = astar.get_astar_grid_path(start, goal, treshold=32)

    # if(path is not None):
    #     for node in path:
    #         print(node)    
    
    # Show path - scale image up
    map_display = cv2.resize(map, (640,640), interpolation=cv2.INTER_NEAREST) 
    map_display = cv2.cvtColor(map_display, cv2.COLOR_GRAY2RGB)

    #nCellSize = 32
    nCellSize = 10
    for node in path:
        x, y = node[1]*nCellSize, node[0]*nCellSize
        cv2.rectangle(map_display, (x,y), (x+nCellSize,y+nCellSize), (0,0,255), -1) 
        
    cv2.imshow("Path", map_display)
    cv2.waitKey(0)