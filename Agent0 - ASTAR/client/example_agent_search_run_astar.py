import math

import client
import ast
import random
import copy


# AUXILIAR

class Queue:
    def __init__(self):
        self.queue_data = []

    def isEmpty(self):
        if len(self.queue_data) == 0:
            return True
        else:
            return False

    def pop(self):
        return self.queue_data.pop(0)

    def insert(self, element):
        return self.queue_data.append(element)

    def getQueue(self):
        return self.queue_data


# SEARCH AGENT
class Node:
    def __init__(self, state, parent, action, path_cost):
        self.state = state
        self.parent = parent
        self.neighbors = []
        self.action = action
        self.path_cost = path_cost  # g
        self.f = 0
        self.h = 0  # heuristica

    def getState(self):
        return self.state

    def getParent(self):
        return self.parent

    def getAction(self):
        return self.action

    def getPathCost(self):
        return self.path_cost

    def setPathCost(self, new_cost):
        self.path_cost = new_cost

    def heuristic(self, goal):
        dx = abs(self.state[0] - goal[0])
        dy = abs(self.state[1] - goal[1])
        self.h = dx + dy

    def setF(self):
        self.f = self.path_cost + self.h

    def getF(self):
        return self.f


class Agent:
    def __init__(self):
        self.c = client.Client('127.0.0.1', 50001)
        self.res = self.c.connect()
        random.seed()  # To become true random, a different seed is used! (clock time)
        self.visited_nodes = Queue()
        self.frontier_nodes = Queue()
        self.weightMap = []
        self.goalNodePos = (0, 0)
        self.state = (0, 0)
        self.maxCoord = (0, 0)
        self.obstacles = []

    def getConnection(self):
        return self.res

    # NEW NEW
    def getDirection(self):
        msg = self.c.execute("info", "direction")
        dir = msg  # ast.literal_eval(msg)
        # test
        print('Dir is:', dir)
        return dir

    def getGoalPosition(self):
        msg = self.c.execute("info", "goal")
        goal = ast.literal_eval(msg)
        # test
        print('Goal is located at:', goal)
        return goal

    def markVisited(self, state):
        self.c.execute("mark", str(state[0]) + "," + str(state[1]) + "_green")

    def markFrontier(self, state):
        self.c.execute("mark", str(state[0]) + "," + str(state[1]) + "_red")

    def markMove(self, state):
        self.c.execute("mark", str(state[0]) + "," + str(state[1]) + "_blue")

    def getSelfPosition(self):
        msg = self.c.execute("info", "position")
        pos = ast.literal_eval(msg)
        # test
        print('Received agent\'s position:', pos)
        return pos

    def getWeightMap(self):
        msg = self.c.execute("info", "map")
        w_map = ast.literal_eval(msg)
        # test
        print('Received map of weights:', w_map)
        return w_map

    def getPatchCost(self, pos):
        return self.weightMap[pos[0]][pos[1]]

    def getMaxCoord(self):
        msg = self.c.execute("info", "maxcoord")
        max_coord = ast.literal_eval(msg)
        # test
        print('Received maxcoord', max_coord)
        return max_coord

    def getObstacles(self):
        return self.obstacles

    def constructObstacles(self):
        msg = self.c.execute("info", "obstacles")
        self.obstacles = ast.literal_eval(msg)
        # test
        # print('Received map of obstacles:', obst)

    def step(self, pos, action):
        if action == "east":
            if pos[0] + 1 < self.maxCoord[0]:
                new_pos = (pos[0] + 1, pos[1])
            else:
                new_pos = (0, pos[1])

        if action == "west":
            if pos[0] - 1 >= 0:
                new_pos = (pos[0] - 1, pos[1])
            else:
                new_pos = (self.maxCoord[0] - 1, pos[1])

        if action == "south":
            if pos[1] + 1 < self.maxCoord[1]:
                new_pos = (pos[0], pos[1] + 1)
            else:
                new_pos = (pos[0], 0)

        if action == "north":
            if pos[1] - 1 >= 0:
                new_pos = (pos[0], pos[1] - 1)
            else:
                new_pos = (pos[0], self.maxCoord[1] - 1)
        return new_pos

    def getNode(self, parent_node, action):
        state = self.step(parent_node.getState(), action)
        pathCost = parent_node.getPathCost() + self.getPatchCost(state)
        return Node(state, parent_node, action, pathCost)

    def printNodes(self, type, nodes, i):
        print(type, " (round ", i, " )")
        print("state | path cost")
        for node in nodes.getQueue():
            print(node.getState(), "|", node.getPathCost())

    def printPath(self, node):
        n = node
        n_list = []
        while n.getPathCost() != 0:
            n_list.insert(0, [n.getState(), n.getPathCost()])
            n = n.getParent()
        n_list.insert(0, [self.getSelfPosition(), 0])
        print("Final Path", n_list)

    def setObstacle(self, x, y):
        self.obstacles[x][y] = 1

    def run_algorithm(self, current, direction=None):
        open_set = []
        closed_set = []
        found = None

        if direction is not None:
            current = Node(ast.literal_eval(current), None, "", 0)
            if direction == "north":
                self.setObstacle(current.getState()[0], current.getState()[1]-1)
            elif direction == "east":
                self.setObstacle(current.getState()[0] + 1, current.getState()[1])
            elif direction == "south":
                self.setObstacle(current.getState()[0], current.getState()[1] + 1)
            elif direction == "west":
                self.setObstacle(current.getState()[0] - 1, current.getState()[1])

        open_set.append(current)
        while len(open_set) > 0 and found is None:
            winner = 0
            for i in range(len(open_set)):
                if open_set[i].f < open_set[winner].f:
                    winner = i

            current = open_set[winner]

            if current.getState() == self.goalNodePos:
                found = open_set[winner]
                self.exe(found)

            open_set.remove(current)
            closed_set.append(current)
            self.markVisited(current.getState())

            for dir in ["north", "east", "south", "west"]:
                neighbor = self.getNode(current, dir)

                if self.getObstacles()[neighbor.getState()[0]][neighbor.getState()[1]] == 1:
                    closed_set.append(neighbor)

                list_visited = []
                list_frontier = []
                for n in closed_set:
                    list_visited.append(n.getState())
                for u in open_set:
                    list_frontier.append(u.getState())

                if neighbor.getState() not in list_visited:
                    temp_g = current.getPathCost() + 1
                    if neighbor.getState() in list_frontier:
                        if temp_g < neighbor.getPathCost():
                            neighbor.setPathCost(temp_g)
                    else:
                        neighbor.setPathCost(temp_g)
                        open_set.append(neighbor)
                        self.markFrontier(neighbor.getState())
                    neighbor.heuristic(self.goalNodePos)
                    neighbor.setF()

            else:
                print("Didn't find the GOAL!!!")

    def run(self):

        # Get information of the weights for each step in the world ...
        self.weightMap = self.getWeightMap()
        # Get max coordinates
        self.maxCoord = self.getMaxCoord()
        # Get the position of the Goal
        self.goalNodePos = self.getGoalPosition()
        # Get the initial position of the agent
        self.state = self.getSelfPosition()
        self.constructObstacles()
        # Add first node (root)
        current = Node(self.state, None, "", 0)
        self.run_algorithm(current)

    # Return the direction to which the robot must turn based on the differences between coordinates from
    # actual position and next position
    def getNextDirection(self, pos, next_pos):
        dir = None
        # North or South
        if (pos[0] == next_pos[0]):
            if (next_pos[1] == pos[1] - 1) or (pos[1] - next_pos[1] == -1 * (self.maxCoord[1] - 1)):
                dir = "north"
            elif (next_pos[1] == pos[1] + 1) or (pos[1] - next_pos[1] == (self.maxCoord[1] - 1)):
                dir = "south"
        # East or West
        elif (pos[1] == next_pos[1]):
            if (next_pos[0] == pos[0] + 1) or (pos[0] - next_pos[0] == (self.maxCoord[0] - 1)):
                dir = "east"
            elif (next_pos[0] == pos[0] - 1) or (pos[0] - next_pos[0] == -1 * (self.maxCoord[0] - 1)):
                dir = "west"
        return dir

    # Return the number of turns and the direction of the turns based on actual direction and desired direction
    def getTurns(self, direction, desired_direction):
        # Return directions
        if direction == "north" and desired_direction == "east":
            return ["right"]
        if direction == "north" and desired_direction == "west":
            return ["left"]
        if direction == "north" and desired_direction == "south":
            return ["right", "right"]

        if direction == "south" and desired_direction == "east":
            return ["left"]
        if direction == "south" and desired_direction == "west":
            return ["right"]
        if direction == "south" and desired_direction == "north":
            return ["right", "right"]

        if direction == "east" and desired_direction == "north":
            return ["left"]
        if direction == "east" and desired_direction == "south":
            return ["right"]
        if direction == "east" and desired_direction == "west":
            return ["right", "right"]

        if direction == "west" and desired_direction == "north":
            return ["right"]
        if direction == "west" and desired_direction == "south":
            return ["left"]
        if direction == "west" and desired_direction == "east":
            return ["left", "left"]
        return []

    def exe(self, final_node=None):
        actual_dir = self.getDirection()
        actual_pos = self.getSelfPosition()
        steps = []
        actual_node = final_node
        # Follow from the goal leaf to root...
        while actual_node.getPathCost() != 0:
            self.markMove(actual_node.getState())
            steps.insert(0, [actual_node.getState(), actual_node.getPathCost()])
            actual_node = actual_node.getParent()
        steps.insert(0, [actual_pos, 0])

        print("Final Path", steps)

        actions = []
        fim = False
        i = 0
        print("Length of steps:", len(steps))
        while fim == False:
            actual_step = steps[i]
            next_step = steps[i + 1]
            print("Actual step:", actual_step)
            print("Next step:", next_step)
            next_dir = self.getNextDirection(actual_step[0], next_step[0])
            turns = self.getTurns(actual_dir, next_dir)
            for turn_action in turns:
                actions.append(turn_action)
            actions.append("forward")
            i = i + 1
            if i >= len(steps) - 1:
                fim = True
            else:
                actual_dir = next_dir
        print("Actions:", actions)
        for action in actions:
            if action == "forward":
                view_front = self.c.execute("info", "view")
                if view_front[2:10] == 'obstacle':
                    self.run_algorithm(self.c.execute("info", "position"), self.c.execute("info", "direction"))
                else:
                    self.c.execute("command", action)
            else:
                self.c.execute("command", action)

# STARTING THE PROGRAM:
def main():
    print("Starting client!")
    ag = Agent()
    if ag.getConnection() != -1:
        ag.run()


main()
