# This program is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.

# This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.

# You should have received a copy of the GNU General Public License along with this program. If not, see <https://www.gnu.org/licenses/>.

# please note that I asked chatgpt to implement a* search and it gave me the idea of using a priority queue and the most of following code is written by chatgpt.
# I made sure that code is working with simple data entry and modifications and provided explaination
# copyright 2024, Tony George.
import heapq
# heapq is a module to construct heaps. we can use it to make a priority queue to enhance search iterations

class Node:
    def __init__(self, city, parent=None, cost=0, heuristic=0):
        self.city = city
        self.parent = parent
        self.cost = cost # cost is the distance from the parent to current city
        self.heuristic = heuristic # heuristic is the straight line distance between current city and the goal

    def __lt__(self, other):
        return (self.cost + self.heuristic) < (other.cost + other.heuristic)
    # "less than", a function to help heapq compare nodes to sort them in the priority queue according to total cost

def astar_search(initial_city, goal_city, graph):
    frontier = []
    # frontier is our priority queue
    heapq.heappush(frontier, Node(initial_city, None, 0, heuristic(initial_city)))
    # heappush is a function that inserts a new node to the priority queue, we pass the heap itself to it and the new element we wish to insert
    explored = set()
    # a set to remember what cities we already visited

    while frontier:
    # this loop continues as long as our priority queue is not empty
        node = heapq.heappop(frontier)
        if node.city == goal_city:
            return node

        explored.add(node.city)

        for neighbor, distance in graph[node.city]:
            # this loop iterates over all neighbours of the node we took from the frontier
            if neighbor not in explored:
                heapq.heappush(frontier, Node(neighbor, node, node.cost + distance, heuristic(neighbor)))

    return None

# graph representing connections between cities and their distances
graph = {
    'arad': [('zerind', 75), ('sibiu', 140), ('timisoara', 118)],
    'bucharest': [('giurgiu', 90), ('ptesti', 101), ('fagaras', 211), ('urziceni', 85)],
    'craiova': [('dobreta', 120), ('rimmieu vilcea', 146), ('ptesti', 138)],
    'dobreta': [('mehadia', 75), ('craiova', 120)],
    'eforie': [('hirsova', 86)],
    'fagaras': [('sibiu', 99), ('bucharest', 211)],
    'giurgiu': [('bucharest', 90)],
    'hirsova': [('eforie', 86), ('urziceni', 98)],
    'iasi': [('neamt', 87), ('vaslui', 92)],
    'lugoj': [('mehadia', 70), ('timisoara', 111)],
    'mehadia': [('lugoj', 70), ('dobreta', 75)],
    'neamt': [('iasi', 87)],
    'oradea': [('zerind', 71), ('sibiu', 151)],
    'ptesti': [('bucharest', 101), ('craiova', 138), ('rimmieu vilcea', 97)],
    'rimmieu vilcea': [('ptesti', 97), ('sibiu', 80), ('craiova', 146)],
    'sibiu': [('fagaras', 99), ('arad', 140), ('rimmieu vilcea', 80), ('oradea', 151)],
    'timisoara': [('lugoj', 111), ('arad', 118)],
    'urziceni': [('hirsova', 98), ('bucharest', 85), ('vaslui', 142)],
    'vaslui': [('urziceni', 142), ('iasi', 92)],
    'zerind': [('oradea', 71), ('arad', 75)]
}

def heuristic(city):
    # straigh-line distance to bucharest
    distances = {
      'arad': 366,
      'bucharest': 0,
      'craiova': 160,
      'dobreta': 242,
      'eforie': 161,
      'fagaras': 176,
      'giurgiu': 77,
      'hirsova': 151,
      'iasi': 226,
      'lugoj': 244,
      'mehadia': 241,
      'neamt': 234,
      'oradea': 380,
      'ptesti': 10,
      'rimmieu vilcea': 193,
      'sibiu': 253,
      'timisoara': 329,
      'urziceni': 80,
      'vaslui': 199,
      'zerind': 374
    }
    return distances[city]

initial_city = 'arad'
goal_city = 'bucharest'

result_node = astar_search(initial_city, goal_city, graph)

# Retrieve path from the result node
path = []
current_node = result_node
while current_node:
    path.append(current_node.city)
    current_node = current_node.parent

# Reverse the path to get it from initial city to goal city
path.reverse()

print("Shortest Path:")
print(path)
