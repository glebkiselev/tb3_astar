from heapq import *

graph = {
    'A':[(2, 'M'), (3, 'P')],
    'M':[(2, 'A'), (2, 'N')],
    'N': [(2, 'M'), (2, 'B')],
    'P': [(3, 'A'), (4, 'B')],
    'B': [(4, 'P'), (2, 'N')]}


def heuristic(a, b):
    # return Manhatten distance
    for el1 in graph[a]:
        for el2 in graph[b]:
            if el1[1] == el2[1]:
                return el1[0] + el2[0]
        else:
            return 0
    # return abs(a[0] - b[0]) + abs(a[1] - b[1])

def dijkstra(start, goal, graph):
    queue = []
    heappush(queue, (0, start))
    cost_visited = {start:0} # стоимость посещенных вершин
    visited = {start:None} # dict стоимости от начальной вершины

    while queue:
        cur_cost, cur_node = heappop(queue) # вынимаем вершины с минимальной ценой
        if cur_node == goal:
            queue = []
            continue

        next_nodes = graph[cur_node] # пересчитываем новую стоимость перемещения от текущей вершины
        for next_node in next_nodes:
            neigh_cost, neigh_node = next_node
            new_cost = cost_visited[cur_node] + neigh_cost

            if neigh_node not in cost_visited or new_cost < cost_visited[neigh_node]:
                # priority = new_cost +heuristic(neigh_node, goal)
                # heappush(queue, (priority, neigh_node))
                heappush(queue, (new_cost, neigh_node))
                cost_visited[neigh_node] = new_cost
                visited[neigh_node] = cur_node
    return visited

start = 'A'
goal = 'B'

visited = dijkstra(start, goal, graph)

cur_node = goal

print('path from {0} to {1}: \n {2}'.format(start, goal, goal), end=' ')

while cur_node != start:
    cur_node = visited[cur_node]
    print(f'-----> {cur_node}', end = ' ')