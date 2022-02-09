import json
import math
from queue import PriorityQueue


def ucs(start_node, end_node, adj, distances):
    # Backtrack through the parents to generate the path
    def backtrack(parent_dict, node):
        path = [node]
        while parent_dict[node] is not None:
            node = parent_dict[node]
            path.append(node)
        return path

    # Setup
    queue = PriorityQueue()
    parent = {start_node: None}
    total_dist = {start_node: 0}
    visited = set()
    queue.put((0, start_node))
    while not queue.empty():
        dist, elem = queue.get()
        # If it has been visited
        if elem in visited:
            continue

        visited.add(elem)
        # If the end node has been reached, backtrack to get the path
        if elem == end_node:
            path = backtrack(parent, elem)
            return path, dist
        for neighbour in adj[elem]:
            n_dist = dist + distances[elem + "," + neighbour]
            if neighbour not in visited and (neighbour not in total_dist or total_dist[neighbour] > n_dist):
                queue.put((n_dist, neighbour))
                total_dist[neighbour] = n_dist
                parent[neighbour] = elem
    return None, None



def ucsWithCost(start_node, end_node, adj, distances, costs, budget):
    # Backtrack through the parents to generate the path
    def backtrack(parent_dict, pair):
        node, _ = pair
        path = [node]
        while parent_dict[pair] is not None:
            pair = parent_dict[pair]
            node, _ = pair
            path.append(node)
        return path

    # Setup
    queue = PriorityQueue()
    parent = {(start_node, 0): None}
    total_dist = {(start_node, 0): 0}
    visited = set()
    min_dist = {}
    min_cost = {}
    queue.put((0, (start_node, 0)))
    while not queue.empty():
        dist, (elem, cost) = queue.get()
        # If it has been visited
        if elem in min_dist \
                and elem in min_cost \
                and min_dist[elem] <= dist \
                and min_cost[elem] <= cost:
            continue
        if elem not in min_dist\
                or min_dist[elem] > dist:
            min_dist[elem] = dist

        if elem not in min_cost\
                or min_cost[elem] > cost:
            min_cost[elem] = cost
        visited.add((elem, cost))
        # If the end node has been reached, backtrack to get the path
        if elem == end_node:
            path = backtrack(parent, (elem, cost))
            return path, dist, cost
        for neighbour in adj[elem]:
            n_dist = dist + distances[elem + "," + neighbour]
            n_cost = cost + costs[elem + "," + neighbour]
            neighbour_node = (neighbour, n_cost)
            if neighbour_node not in visited \
                    and (neighbour_node not in total_dist) \
                    and (n_cost <= budget):
                queue.put((n_dist, neighbour_node))
                total_dist[neighbour_node] = n_dist
                parent[neighbour_node] = (elem, cost)
    return None, None, None

def aStar(start_node, end_node, adj, distances, costs, budget, coords):
    # Backtrack through the parents to generate the path
    def backtrack(parent_dict, pair):
        node, _ = pair
        path = [node]
        while parent_dict[pair] is not None:
            pair = parent_dict[pair]
            node, _ = pair
            path.append(node)
        return path

    # Euclidean distance
    def euclDist(node1, node2):
        x1, y1 = coords[node1]
        x2, y2 = coords[node2]
        return math.sqrt((x1 - x2)**2 + (y1 - y2)**2)

    # Ensure A* Heuristic is consistent
    def Verify():
        for k,v in distances.items():
            x, y = k.split(",")
            if euclDist(x,y) > v:
                print("Invalid metric! Key: ", k, " Value: ", v)

    # Setup
    Verify()
    queue = PriorityQueue()
    parent = {(start_node, 0): None}
    total_dist = {(start_node, 0): 0}
    visited = set()
    min_dist = {}
    min_cost = {}
    queue.put((0, (start_node, 0)))
    while not queue.empty():
        _, nd = queue.get()
        (elem, cost) = nd
        # If it has been visited
        if elem in min_dist \
                and elem in min_cost \
                and min_dist[elem] <= total_dist[nd] \
                and min_cost[elem] <= cost:
            continue
        if elem not in min_dist\
                or min_dist[elem] > total_dist[nd]:
            min_dist[elem] = total_dist[nd]

        if elem not in min_cost\
                or min_cost[elem] > cost:
            min_cost[elem] = cost
        visited.add((elem, cost))
        # If the end node has been reached, backtrack to get the path
        if elem == end_node:
            path = backtrack(parent, nd)
            return path, total_dist[nd], cost
        for neighbour in adj[elem]:
            n_dist = total_dist[nd] + distances[elem + "," + neighbour]
            n_cost = cost + costs[elem + "," + neighbour]
            neighbour_node = (neighbour, n_cost)
            if neighbour_node not in visited \
                    and (neighbour_node not in total_dist) \
                    and (n_cost <= budget):
                queue.put((n_dist + euclDist(neighbour, end_node), neighbour_node))
                total_dist[neighbour_node] = n_dist
                parent[neighbour_node] = nd
    return None, None, None


if __name__ == "__main__":
    start = "1"
    end = "50"
    max_cost = 287932
    with open("./G.json") as GF:
        G = json.load(GF)
    with open("./Cost.json") as CostF:
        Cost = json.load(CostF)
    with open("./Dist.json") as DistF:
        Dist = json.load(DistF)
    with open("./Coord.json") as CoordF:
        Coord = json.load(CoordF)

    print("==============================\nTask 1:\n==============================")
    path, dist = ucs(start, end, G, Dist)
    ans = path.pop()
    while len(path) > 0:
        ans += "->" + path.pop()
    print("Shortest Path: ", ans)
    print("Shortest Distance: ", dist)

    print("==============================\nTask 2:\n==============================")
    path, dist, cost = ucsWithCost(start, end, G, Dist, Cost, max_cost)
    ans = path.pop()
    while len(path) > 0:
        ans += "->" + path.pop()
    print("Shortest Path: ", ans)
    print("Shortest Distance: ", dist)
    print("Cost: ", cost)

    print("==============================\nTask 3:\n==============================")
    path, dist, cost = aStar(start, end, G, Dist, Cost, max_cost, Coord)
    ans = path.pop()
    while len(path) > 0:
        ans += "->" + path.pop()
    print("Shortest Path: ", ans)
    print("Shortest Distance: ", dist)
    print("Cost: ", cost)

