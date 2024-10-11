# (c) robotics.snowcron.com
# Use: MIT license
 
import math
import cv2
import numpy as np
from collections import defaultdict
import os
import json

class AStarGraph:

    def __init__(self):
        self.adj_list = None
        self.nodes = None
        
    # --- Helper, currently not used

    # Function to build the graph
    def build_graph(self):
        edges = [
            ["A", "B"], ["A", "E"], 
            ["A", "C"], ["B", "D"],
            ["B", "E"], ["C", "F"],
            ["C", "G"], ["D", "E"]
        ]
        graph = defaultdict(list)
        
        # Loop to iterate over every 
        # edge of the graph
        for edge in edges:
            a, b = edge[0], edge[1]
            
            # Creating the graph 
            # as adjacency list
            graph[a].append(b)
            graph[b].append(a)
        return graph
 
    # if __name__ == "__main__":
    #     graph = build_graph()
        
    #     print(graph)

    # Output: 
    # {
    # 'G': ['C'], 
    # 'F': ['C'], 
    # 'E': ['A', 'B', 'D'], 
    # 'A': ['B', 'E', 'C'], 
    # 'B': ['A', 'D', 'E'], 
    # 'D': ['B', 'E'], 
    # 'C': ['A', 'F', 'G']
    # }        

    # ---

    def heuristic(self, n1, n2):
        x1, y1 = self.nodes[n1]
        x2, y2 = self.nodes[n2]  
        #print("heuristic(", n1, n2, "):", math.sqrt((x1 - x2)**2 + (y1 - y2)**2))
        return math.sqrt((x1 - x2)**2 + (y1 - y2)**2)

    def get_neighbors(self, node, treshold):
        neighbors = []
        for neighbour in self.adj_list[node]:
            if(neighbour[1] > treshold):
                neighbors.append(neighbour)
        return neighbors

    def get_astar_graph_path(self, start, goal, treshold):

        openset = set()
        openset.add(start)

        closedset = set()
        
        came_from = {}
        
        g_score = { start: 0 }
        f_score = { start: self.heuristic(start, goal) }

        while openset:

            current = min(openset, key=lambda x: f_score[x])
            
            if current == goal:
                path = []
                while current in came_from:
                    path.append(current)
                    current = came_from[current]
                path.append(current)
                return path[::-1]

            openset.remove(current)
            closedset.add(current)
            
            for neighbor in self.get_neighbors(current, treshold):
                tentative_g_score = g_score[current] + self.heuristic(current, neighbor[0]) \
                    * 255.0 / np.maximum(neighbor[1], 1)
                if neighbor[0] in closedset and tentative_g_score >= g_score[neighbor[0]]:
                    continue

                if neighbor[0] not in openset or tentative_g_score < g_score[neighbor[0]]:
                    came_from[neighbor[0]] = current
                    g_score[neighbor[0]] = tentative_g_score
                    f_score[neighbor[0]] = g_score[neighbor[0]] #+ self.heuristic(neighbor[0], goal)
                    #print("f_score[", neighbor[0], "]:", f_score[neighbor[0]])

                    if neighbor[0] not in openset:
                        openset.add(neighbor[0])
            
        return None

    # ---
    # Alternative to copying dictionary in order to have int keys
    # def convert_keys_to_int(ordered_pairs):
    #     result = {}
    #     for key, value in ordered_pairs:
    #          result[int(key)] = value
    #     return result

    # data_int_keys = json.loads(data, object_pairs_hook=convert_keys_to_int)

    # print(data_int_keys)

    def loadGraph(self, strFilePath):
        #print(os.getcwd())
        with open(strFilePath) as f:
            data = f.read()

            #print(data)

            nodes_str, adj_list_str = data.split('adj_list = ')
            nodes_str = nodes_str.split("nodes = ")[1]

            nodes_str = json.loads(nodes_str)  
            adj_list_str = json.loads(adj_list_str)

            nodes = {}
            for k,v in nodes_str.items():
                nodes[int(k)] = v

            adj_list = {}
            for k,v in adj_list_str.items():
                adj_list[int(k)] = v

            # print(nodes)
            # print("***")
            # print(adj_list)

            self.nodes = nodes
            self.adj_list = adj_list

            return nodes, adj_list

# ------

if __name__ == "__main__":

    # Case A
    # nodes = {
    #     1: [50,  50],
    #     2: [200, 200],
    #     3: [300, 300],
    #     4: [400, 400],
    #     5: [200, 300],
    #     6: [300, 200],
    #     7: [200, 400],
    # }

    # adj_list = {
    #     1: [ [2, 255],  [5, 255], [7, 64] ],
    #     2: [ [1, 64], [3, 255] ], 
    #     3: [ [2, 64], [4, 64] ],
    #     4: [ [3, 64], [5, 64],  [7, 64] ],
    #     5: [ [1,255], [3, 64],  [4, 255] ],
    #     6: [],
    #     7: [ [1,128], [4, 64] ],
    # }

    astar = AStarGraph()

    # Case B
    strRoadsGraphFileName = "../maps/nav_graph_world_01.json"
    nodes, adj_list = astar.loadGraph(strRoadsGraphFileName)
    dMinX = float("inf")
    dMinY = float("inf")
    for node in nodes.values():
        dMinX = min(dMinX, node[0])
        dMinY = min(dMinY, node[1])
    for node in nodes.values():
        # +16 for legends to not be cut
        node[0] -= dMinX
        node[1] -= dMinY
        node[0] = (int)(node[0] * 32) + 16
        node[1] = (int)(node[1] * 32) + 16   

    #path = astar.get_astar_graph_path(22, 3, treshold=63)
    path = astar.get_astar_graph_path(23, 22, treshold=63)
    if(path is None):
        print("Path not found")
    else:

        img = 255 * np.ones((640,640,3), dtype=np.uint8)

        for src, arrDst in adj_list.items():
            pt1 = nodes[src]
            for dst in arrDst:
                pt2 = nodes[dst[0]]
                cv2.line(img, pt1, pt2, (0,0,255), 2)

        for i in range(len(path)-1):
            pt1 = nodes[path[i]]
            pt2 = nodes[path[i+1]]
            cv2.line(img, pt1, pt2, (0,255,0), 5)

        font = cv2.FONT_HERSHEY_COMPLEX_SMALL
        fontScale = 0.5
        color = (0, 255, 255)
        thickness = 1
        for k, v in nodes.items():
            cv2.circle(img, (v[0], v[1]), 10, (0,0,255), -1)
            img = cv2.putText(img, str(k), (v[0]-6, v[1]+6), font,  
                   fontScale, color, thickness, cv2.LINE_AA)

        cv2.imshow("Path", img)
        cv2.waitKey(0)
