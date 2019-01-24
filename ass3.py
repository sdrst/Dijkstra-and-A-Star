#Sam Durst
#sd922
#Assignment 3
import math # this imported module is only used to square root the heuristic
import time # to determine how long the program runs, has no effect on the program itself

def main():
    infile = open(input("Please enter a file name: "), 'r')
    start_time = time.time()
    data = infile.read().split()
    nodes = []
    node_coords = [] # 3 arrays, 1 to hold all the nodes, the second holds the x and y coordinate of all the nodes
    node_relations = [] # and the third to hold the costs and edges between all nodes

    num_nodes = int(data[0])

    for i in range(1,num_nodes+1):
        nodes.append(str(i)) # add all the nodes to the array

    for i in range(1,num_nodes*3,3):
        node_coords.append([data[i], float(data[i+1]), float(data[i+2])])  # add all the coordinates

    edge_pos = int(data[0])*3+1
    edges = int(data[edge_pos])

    for i in range(edge_pos+1, edges*3+edge_pos,3):
        node_relations.append([data[i], data[i+1], float(data[i+2])]) # add all the relations

    start = data[edges*3+edge_pos+1] # the starting point is the second to last node in the file
    end = data[edges*3+edge_pos+2] # the ending point is the last node in the file

    matrix = Matrix() # Instance of the matrix class, its data more properly represents a graph than a matrix however

    for i in nodes:
        matrix.add_node(i) # add all the nodes to the matrix (graph)

    for i in node_relations:
        matrix.add_edge(i[0],i[1],i[2]) # connect all the edges between nodes


    result1_distance, result1_path, result1_extra  = shortest_path(matrix, start, end, node_coords , 1)
    result2_distance, result2_path, result2_extra  = shortest_path(matrix, start, end, node_coords , 2)
    result3_distance, result3_path, result3_extra, connecting_point = shortest_path_modified(matrix, start, end)
    additional1 = set(result1_extra)-set(result1_path)
    additional2 = set(result2_extra)-set(result2_path)
    additional3 = set(result3_extra)-set(result3_path)


    print("\nPart 1")
    print("Shortest dijkstra distance: ", result1_distance)
    print("Dijkstra shortest path traveled: ", str(result1_path).strip("[]"))
    print("Additional nodes in solution tree not in shortest path: ", (node_finder(additional1)))
    print("--------------------------------------------------------------")
    print("Part 2")
    print("Shortest A* distance: ", result2_distance)
    print("A* shortest path traveled: ", str(result2_path).strip("[]"))
    print("Additional nodes in solution tree not in shortest path: ", (node_finder(additional2))) # results display
    print("--------------------------------------------------------------")
    print("Part 3")
    print("Shortest modified dijkstra distance: ", result3_distance)
    print("Modified dijkstra shortest path traveled: ", str(result3_path).strip("[]"))
    print("Additional nodes in solution tree not in shortest path: ", node_finder(additional3))
    print("Node where the origin and destination paths met in the middle: ", connecting_point)

    print("\nTime it took this program to run:", time.time()-start_time)
#------------------------------------------------------------------------------#

def dijkstra(matrix, start, end):
    s = [] # array of nodes that have been visited, the compement of the node set
    path = {}
    node_set = set(matrix.nodes) # unordered set

    visited = {start: 0}

    while node_set:
        min_node = None # no minimum node on first run

        for node in node_set:
            if node in visited: # loops until it finds the start node
                if min_node is None:
                    min_node = node # the minimum node on the first node will be the start node
                elif visited[node] < visited[min_node]: # if the cost to get to the next node is less than the current min node
                    min_node = node # the min node now becomes the current node

        s.append(min_node) #creating solution tree
        node_set.remove(min_node) # remove the min node from the set as it has now been visited
        weight = visited[min_node] # the weight will be the cost of getting to the current node

        for edge in matrix.edges[min_node]:

            total_weight = weight + matrix.weights[(min_node, edge)] # the total weight becomes the current weight thus far, plus the cost of the new edge

            if edge not in visited or total_weight < visited[edge]: # if the edge is not in visited we add to visited and path
                visited[edge] = total_weight
                path[edge] = min_node

        if end in visited:
            break # This creates the modified version of dijkstra, without this it would check all the nodes, however it ends when the end node is in the set of visited nodes

    return visited, path, s

def AStar(matrix, start, end, node_coords):
    s = []
    path = {}
    node_set = set(matrix.nodes) #same format as the previous dijkstra in the beginning (unordered set)
    visited = {start: 0}

    while node_set:
        min_node = None

        for node in node_set: # we add a euclidian heuristic based upon the coordinate positions
            if node in visited:
                node_heuristic = math.sqrt((node_coords[int(end)-1][1] - node_coords[int(node)-1][1])**2 + (node_coords[int(end)-1][2] - node_coords[int(node)-1][2])**2) # Sqrt((x2-x1)^2+(y2-y1)^2)

                if min_node is None:
                    min_node = node # for the first run of the loop the min_node heuristic is set here
                    min_node_heuristic = math.sqrt((node_coords[int(end)-1][1] - node_coords[int(min_node)-1][1])**2 + (node_coords[int(end)-1][2] - node_coords[int(min_node)-1][2])**2)

                elif visited[node] + node_heuristic < visited[min_node] + min_node_heuristic:
                    min_node = node
                    min_node_heuristic = math.sqrt((node_coords[int(end)-1][1] - node_coords[int(min_node)-1][1])**2 + (node_coords[int(end)-1][2] - node_coords[int(min_node)-1][2])**2)

                    # the min node heuristic will be updated to the min_node

        s.append(min_node)
        node_set.remove(min_node)
        weight = visited[min_node]

        for edge in matrix.edges[min_node]:
            total_weight = weight + matrix.weights[(min_node, edge)] # The rest follows the same format as dijkstra
            if edge not in visited or total_weight < visited[edge]:
                visited[edge] = total_weight
                path[edge] = min_node

        if end in visited:
            break

    return visited, path, s

def altered_dijkstra(matrix, start, end):
    s = []
    path1 = {}
    path2 = {}   # for the altered_dijkstra an extra path is added
    node_set = set(matrix.nodes) #unordered

    visited1 = {start: 0}
    visited2 = {end: 0} # an extra visited set is added as well

    while node_set:
        min_node1 = None
        min_node2 = None # 2 min nodes as we begin at both the start and end nodes and finish when the visited sets share the same node

        for node in node_set:
            if node in visited1:
                if min_node1 is None:
                    min_node1 = node
                elif visited1[node] < visited1[min_node1]: # same as normal dijkstra for first path (start)
                    min_node1 = node

        s.append(min_node1)
        node_set.remove(min_node1)
        weight = visited1[min_node1]

        for edge in matrix.edges[min_node1]:

            total_weight = weight + matrix.weights[(min_node1, edge)]

            if edge not in visited1 or total_weight < visited1[edge]:
                visited1[edge] = total_weight
                path1[edge] = min_node1

        for i in visited1:
            if i in visited2:
                return visited1, visited2, path1, path2, s, i # checks to see if the sets share a node

        for node in node_set:
            if node in visited2:
                if min_node2 is None:
                    min_node2 = node
                elif visited2[node] < visited2[min_node2]: # repeats the same for the end node
                    min_node2 = node

        s.append(min_node2)
        node_set.remove(min_node2)
        weight = visited2[min_node2]

        for edge in matrix.edges[min_node2]:

            total_weight = weight + matrix.weights[(min_node2, edge)]

            if edge not in visited2 or total_weight < visited2[edge]:
                visited2[edge] = total_weight
                path2[edge] = min_node2

        for i in visited1:
            if i in visited2:
                print("second break")
                return visited1, visited2, path1, path2, s, i # the two paths will meet somewhere in the middle


def shortest_path(matrix, origin, destination, node_coords, choice): # helps display all the data
    if choice == 1:
        visited, paths, s = dijkstra(matrix, origin, destination)
    elif choice == 2:
        visited, paths, s = AStar(matrix, origin, destination, node_coords)

    traveled = []
    end = paths[destination]


    while end != origin:
        traveled.insert(0, end)
        end = paths[end]

    traveled.insert(0, origin)
    traveled.append(destination)


    return round(visited[destination],1), traveled, s


def shortest_path_modified(matrix, origin, destination):
    visited1, visited2, path1, path2, s, i = altered_dijkstra(matrix, origin, destination) # same as above but modified for the dual paths
    traveled = []


    end1 = path1[i]
    end2 = path2[i]

    while end1 != origin:
        traveled.insert(0, end1)
        end1 = path1[end1]

    traveled.append(i)

    while end2 != destination:
        traveled.append(end2)
        end2 = path2[end2]

    traveled.insert(0, origin)
    traveled.append(destination)

    return round(visited1[i] + visited2[i],1), traveled, s, i


def node_finder(x):
    if x==set():
        return "No additional nodes"
    else:
        return len(x)

class Matrix():
    def __init__(self):
        self.nodes = []
        self.edges = {}
        self.weights = {}

    def add_node(self, node):
        self.nodes.append(node)

    def add_edge(self, from_node, to_node, distance):
        self.edges.setdefault(from_node, []).append(to_node)
        self.edges.setdefault(to_node, []).append(from_node)
        self.weights[(from_node, to_node)] = distance
        self.weights[(to_node, from_node)] = distance


main()
