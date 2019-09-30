import bge, bpy
import mathutils

import heapq

cont = bge.logic.getCurrentController()
own = cont.owner

def reconstruct_path(came_from, start, goal):
    current = goal
    path = []
    while current != start:
        path.append(current)
        current = came_from[current]
    path.append(start) # optional
    path.reverse() # optional
    return path

class SimpleGraph:
    def __init__(self):
        self.edges = {}
    
    def neighbors(self, id):
        return self.edges[id]

    def cost(self, from_node, to_node):
        return (own['locations'][from_node] -  own['locations'][to_node]).magnitude   
     
class PriorityQueue:
    def __init__(self):
        self.elements = []
    
    def empty(self):
        return len(self.elements) == 0
    
    def put(self, item, priority):
        heapq.heappush(self.elements, (priority, item))
    
    def get(self):
        return heapq.heappop(self.elements)[1]
    

def heuristic(a, b):
    return((own['locations'][a]-own['locations'][b]).magnitude)

def a_star_search(graph, start, goal):
    frontier = PriorityQueue()
    frontier.put(start, 0)
    came_from = {}
    cost_so_far = {}
    came_from[start] = None
    cost_so_far[start] = 0
    
    while not frontier.empty():
        current = frontier.get()
        
        if current == goal:
            break
        
        for next in graph.neighbors(current):
            new_cost = cost_so_far[current] + graph.cost(current, next)
            if next not in cost_so_far or new_cost < cost_so_far[next]:
                cost_so_far[next] = new_cost
                priority = new_cost + heuristic(goal, next)
                frontier.put(next, priority)
                came_from[next] = current
    
    return came_from, cost_so_far

def main():

    
    if 'Graph' not in own:
        faces = {}
        index = 0
        data = bpy.data.objects[own.name].data
        locations = {}
        for face in data.polygons:
            links = []
            
            faceVerts = face.vertices
            place = None
            for vert in face.vertices:
                if place==None:
                    place = data.vertices[vert].co
                else:
                    place = (place+data.vertices[vert].co)/2      
            
            locations[index] = place
            
            index2 = 0
            for face2 in data.polygons:
                if face2 != face:
                    face2Verts = face2.vertices
                    for vert in face.vertices:
                    
                        if vert in face2.vertices:
                            if index2 not in links:
                                links.append(index2)
                                
                    
                index2+=1            
            #print(place)
            faces[index] = links
             
            index+=1                         
        print(faces)
        own['locations'] = locations
        own['Graph'] = SimpleGraph()
        own['Graph'].edges = faces
        
        kd = mathutils.kdtree.KDTree(len(locations))
        index = 0
        for entry in locations:
            #print(locations[entry])
            kd.insert(locations[entry], index)
            index+=1
        kd.balance()
        own['Kd']=kd
             
            
                                    
            
    else:
        player = own.scene.objects['Actor']
        end = own.scene.objects['Target'] 
        
        start = own['Kd'].find(player.worldPosition)[1]
        end = own['Kd'].find(end.worldPosition)[1]
        
        print('start is'+str(start) +" and end is "+str(end))
        path = a_star_search(own['Graph'],start,end)         
        
        truePath = reconstruct_path(path[0], start,end)
        index = 0
        print(truePath)
        for point in truePath:
            if index!=0:
                added = own.scene.addObject('Line',own,1)
                added.worldPosition = own['locations'][point]
                p2 = own['locations'][truePath[index-1]]
                v2 = added.getVectTo(p2)
                added.alignAxisToVect(v2[1],1,1)
                added.localScale = [1,v2[0],1]
            index+=1    
                
                
main()
