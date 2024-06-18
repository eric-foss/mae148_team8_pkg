import matplotlib.pyplot
import numpy as np
import shapely

import matplotlib as plt
import shapely.ops
import shapely.plotting
#from mae148_team8_pkg.coord_handling import initialize_wksp_obs
from coord_handling import initialize_wksp_obs

#Pick a location for the car to navigate towards
#Need to be converted to UTM

#EBU 1 :
#goal_location = [32.881275,-117.235466]

#Base of Geisel Steps:
#goal_location = [32.881118,-117.235901]

class Workspace():
    def __init__(self, workspace, obstacle_dict,start_location,goal_location,boundary_type = 0, buffer = 0.5, origin=None):
        self.workspace = workspace
        self.obstacle_dict = obstacle_dict
        self.origin = origin
        self.ws_rect = self.getRectBounds(self.workspace)
        self.obs_dict_hull = self.getConvexHullDict(self.obstacle_dict)
        self.obs_dict_rect = self.getRectBoundsDict(self.obstacle_dict)
        
        if boundary_type == 0:
            self.obs_union = self.getUnaryUnion(self.obs_dict_rect)
        elif boundary_type == 1:
            self.obs_union = self.getUnaryUnion(self.obs_dict_hull)
        else:
            return 'Please specify either 0 or 1 for rect or hull bounds.'
        
        self.free_workspace = self.getFreeWorkspace(self.ws_rect,self.obs_union)
        self.triangles = self.getDecomposition(self.free_workspace)
        self.start_location = start_location
        self.goal_location =  goal_location
        self.buffer = buffer
        self.nodes,self.edges,self.adj_table,self.weights = self.getGraph(self.triangles,self.obs_union,self.start_location,self.goal_location,self.buffer)
        self.path = self.getBestFSPath(self.nodes,self.adj_table)
        self.path_coords = self.extractPathCoords(self.nodes,self.path)

    #Defines the set of coordinates wrt the origin
    def getLocalFrame(self, coords):
        local_frame = []
        for i in range(len(coords)):
            if coords[i] == origin:
                local_frame.append(np.array([0,0]))
            else:
                local_frame.append(np.subtract(coords[i],self.origin))
            local_frame[i] = np.ndarray.tolist(local_frame[i])
    
        return local_frame


    def getConvexHull(self,vertices):
        points = shapely.Polygon(vertices)
        hull = shapely.convex_hull(points)
        
        return hull 
    
    def getConvexHullDict(self,obs_dict):
        obs_dict_hull = {}
        for k,v in obs_dict.items():
            obs_dict_hull[k] = self.getConvexHull(v)
        
        return obs_dict_hull
    
    def getRectBounds(self, vertices):
        points = shapely.Polygon(vertices)
        rect = shapely.envelope(points)
        
        return rect
    
    def getRectBoundsDict(self,obs_dict):
        obs_dict_rect = {}
        for k, v in obs_dict.items():
                obs_dict_rect[k] = self.getRectBounds(v)
        
        return obs_dict_rect
    

    def getUnaryUnion(self, obstacle_dict):
        obstacle_list = list(obstacle_dict.values())
        obs_union = shapely.ops.unary_union(obstacle_list)

        return obs_union
    
    def getFreeWorkspace(self,workspace_rect,obstacle_union):
        Q_free = workspace_rect.difference(obstacle_union)

        return Q_free


    def getDecomposition(self, Q_free):
        
        regions = shapely.ops.triangulate(Q_free)
        triangles = []

        for shape in regions:
            if shape.within(Q_free) == True:
                triangles.append(shape)
                #shapely.plotting.plot_polygon(shape)
        
        return triangles


    def getGraph(self,triangles,obstacle_union,start_location,goal_location,buffer=0.5):
        nodes = []
        edges = []
        adj_table = {}
        weights = {}

        try:
            nodes.insert(0,start_location)
        except:
            print('Start not Initialized')

        for tri in triangles:
            nodes.append(tri.centroid)

        try:
            nodes.append(goal_location)
        except:
            print('Goal not initialized')
        
        for node in nodes:
            adj_table[nodes.index(node)] = []
            weights[nodes.index(node)] = []
            for point in nodes:
                if node != point:
                    try:
                        line = shapely.LineString([node,point])
                        if line.intersects(obstacle_union.buffer(buffer)) == False:
                            edges.append(line)
                            adj_table[nodes.index(node)].append(nodes.index(point))
                            weights[nodes.index(node)].append(line.length)
                    except:
                        pass
        return nodes, edges, adj_table,weights

    def getBestFSPath(self,nodes,adj_table):
        queue = [0]
        visited = []
        parent_dict = {}    

        for vertex in adj_table:
            
            if vertex == 0:
                parent_dict[vertex] = 0
            else:
                parent_dict[vertex] = None
        j = 0
        while queue != []:
            dists_to_goal = []
            for item in queue:
                if nodes[item] != nodes[-1]:
                    dists_to_goal.append(shapely.LineString((nodes[item],nodes[-1])).length)
            
            min_dist = []
            for dist in dists_to_goal:
                if min_dist == []:
                    min_dist.append(dist)
                elif dist < min_dist[0]:
                    min_dist.append(dist)
                    min_dist.pop(0)

            index_min = dists_to_goal.index(min_dist[0])

            vertex_ind = queue.pop(index_min)
            visited.append(vertex_ind)
            
            for node in adj_table[vertex_ind]:
                if parent_dict[node] == None:
                    parent_dict[node] = vertex_ind
                    queue.append(node)

        ind_f = nodes.index(nodes[-1])
        g = [ind_f]
        j = 0
        parent_pointer = -1
        for i in range(len(nodes)):
            if j  !=  len(nodes) and g[0] != 0:
                if parent_pointer == -1:
                    parent_pointer = parent_dict[ind_f]
                    g.insert(0,parent_pointer)
                elif parent_pointer == 0:
                    break
                else:
                    parent_pointer = parent_dict[parent_pointer]
                    g.insert(0,parent_pointer)
                j +=1
                if j == len(nodes) and g[0] != 0:
                    return 'No Path Found'
        return g

    def extractPathCoords(self,nodes,path):
        coords = []
        for point in path:
             x = nodes[point].x 
             y = nodes[point].y
             coords.append((x,y))
        return coords

    def getCurrentPosition(self,tuple_xy):
        current_position = shapely.Point(tuple_xy[0],tuple_xy[1])
        return current_position
    
    def isNearGoal(self,current_position_def,buffer=1):
        goal_position = self.nodes[-1]
        polygoal=goal_position.buffer(buffer)
        if shapely.instersects(current_position_def,polygoal):
            return True
        else:
            return False

if __name__ == "__main__":
    #Example Workspace
    master = initialize_wksp_obs("test3.txt")
    workspace = master['wksp']
    origin = master['origin']
    obs_dict = master['obs_dict']

    start_location = shapely.Point((-.185,-11))
    goal_location = shapely.Point((0,0))

    current_position = (4,17)

    ws = Workspace(workspace,obs_dict,start_location,goal_location,boundary_type=1)

    print(ws.path)
    print(ws.path_coords)

    #for line in ws.getCTE(current_position):
    #    shapely.plotting.plot_line(line)

    shapely.plotting.plot_polygon(ws.free_workspace)

    #for node in ws.nodes:
    #    shapely.plotting.plot_points(node)

    #for edge in ws.edges:
    #    shapely.plotting.plot_line(edge)

    for node in ws.path_coords:
        shapely.plotting.plot_points(shapely.Point(node))

    plt.pyplot.show()