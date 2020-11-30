#!/usr/bin/python3
import os

import rospy

from geometry_msgs.msg import Twist
from nav_msgs.msg import OccupancyGrid

from heapq import *
import pickle


class AstarNode:

    def __init__(self):
        rospy.init_node('Astar')
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        rospy.wait_for_message('/map',OccupancyGrid)
        self.sub = rospy.Subscriber('/map',OccupancyGrid,self.update)
        rospy.on_shutdown(self.stop)

    def heuristic(self, vert_id):
        # return Manhatten distance
        vert = None
        for v in self.map_rep:
            if v.id == vert_id:
                vert = v
                break
        return abs(vert.x - self.goal_node.x) + abs(vert.y - self.goal_node.y)

    def make_graph(self, map_rep):
        # create dict where all costs to neightbours are computed (only left-right-up-down)
        graph = {}
        for node in map_rep:
            if node.status != -1:
                continue
            if map_rep.index(node) != 0:
                lnode = map_rep[map_rep.index(node)-1]
                if lnode:
                    if lnode.x == node.x - 1 and lnode.y == node.y:
                        ptn = 1
                        if lnode.status == 100: ptn = 10000
                        graph.setdefault(node.id, []).append((ptn, lnode.id))
            if map_rep.index(node) < len(map_rep)-2:
                rnode = map_rep[map_rep.index(node)+1]
                if rnode.x == node.x + 1 and rnode.y == node.y:
                    ptn = 1
                    if rnode.status == 100: ptn = 10000
                    graph.setdefault(node.id, []).append((ptn, rnode.id))
            for unode in map_rep:
                if unode.y == node.y + 1 and unode.x == node.x:
                    ptn = 1
                    if unode.status == 100: ptn = 10000
                    graph.setdefault(node.id, []).append((ptn, unode.id))
                    break
            for dnode in map_rep:
                if dnode.y == node.y -1 and dnode.x == node.x:
                    ptn = 1
                    if dnode.status == 100: ptn = 10000
                    graph.setdefault(node.id, []).append((ptn, dnode.id))
                    break
            if len(graph) % 1000 == 0:
                print(len(graph))
        return graph

    def search(self, graph):
        print("in search")
        queue = []
        heappush(queue, (0, self.start_node.id))
        cost_visited = {self.start_node.id:0}
        visited = {self.start_node.id:None}

        while queue:
            cur_cost, cur_node = heappop(queue)
            if cur_node == self.goal_node.id:
                queue = []
                continue

            next_nodes = graph[cur_node]
            for next_node in next_nodes:
                neigh_cost, neigh_node = next_node
                new_cost = cost_visited[cur_node] + neigh_cost

                if neigh_node not in cost_visited or new_cost < cost_visited[neigh_node]:
                    priority = new_cost + self.heuristic(neigh_node)
                    heappush(queue, (priority, neigh_node))
                    # heappush(queue, (new_cost, neigh_node))
                    cost_visited[neigh_node] = new_cost
                    visited[neigh_node] = cur_node
        return visited

    def update(self, map_msg):
        self.data=map_msg.data
        self.width=map_msg.info.width
        self.height=map_msg.info.height
        self.originX=int(map_msg.info.origin.position.x)
        self.originY=int(map_msg.info.origin.position.y)
        self.widthNew =self.width + self.originX
        self.HeightNew =self.height + self.originY
        id = 0
        self.map_rep = []
        for i in range(self.originY,self.HeightNew):
            for j in range(self.originX,self.widthNew):
                n=BasicNode()
                n.id=id
                n.status=self.data[id]
                n.x=j
                n.y=i
                self.map_rep.append(n) # append map data to list
                id+=1

        ### set goal
        self.start_node = [el for el in self.map_rep if el.x == 0 and el.y == 0][0] #  x and  y
        if self.start_node.status == 100:
            raise Exception("tb3 in an obstacle at start point!")
        self.goal_node = [el for el in self.map_rep if el.x == 5 and el.y == 1][0]  # big x and big y <------change goal here
        if self.goal_node.status == 100:
            raise Exception("tb3 in an obstacle at goal point!")
        self.start_node.g = 0
        self.start_node.h = self.heuristic(self.start_node.id)

        if not "graph.pkl" in os.listdir("."):
            graph = self.make_graph(self.map_rep)
            with open("graph.pkl", 'wb') as fp:
                pickle.dump(graph, fp)
        else:
            with open("graph.pkl", "rb") as fp:
                graph = pickle.load(fp)

        path = self.search(graph)
        print(path)


    def run(self):
        rospy.spin()

    def stop(self):
        vel_msg = Twist()
        self.pub.publish(vel_msg)

class BasicNode:
    """
    init a node saver for each cell
    f(x) = g(x)+h(x)
    g(x)+h(x) <= g(y)+h(y)
    """

    def __init__ (self, id = 0, x = 0, y = 0, g = 0, h = 0, f = 0, parent = 0, status = 0):
        self.id = id
        self.x = x
        self.y = y
        self.g = g # cost to node
        self.h = h # cost from node
        self.f = f # full cost
        self.parent = parent
        self.status = status
    def __str__(self):
        return "id: {0}, x: {1}, y: {2}," \
               " g: {3}, h: {4}, f: {5}, parent: {6}, status: {7}".format(self.id,
                                                                          self.x, self.y, self.g, self.h,
                                                                          self.f, self.parent, self.status)

def main():
    astar_node = AstarNode()
    astar_node.run()


if __name__ == '__main__':
    main()