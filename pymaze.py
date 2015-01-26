from pygame import *

import random

from Queue import PriorityQueue


class AStar:
    def __init__(self, maze, start, goal):
        self.maze = maze
        self.start = start
        self.goal = goal

        self.cost_to = {start: 0}
        self.came_from = {start: None}

        self.closed_set = set()
        self.frontier = PriorityQueue()
        self.frontierSet = set()  # same contents as frontier, but stored as a set for O(1) lookup

        self.frontier.put((0, start))
        self.frontierSet.add(start)

    def grow(self):
        # remove and return the lowest-cost item from the frontier
        current = self.frontier.get()
        if current == self.goal:
            return current

        self.closed_set.add(current)
        self.frontierSet.remove(current)

        for neighbor in self.maze.get_neighbors(current):
            self.cost_to[neighbor] = self.cost_to[current] + 1
            self.came_from[neighbor] = current
            self.frontier.put((self.cost_to[neighbor] + self.min_cost_to_goal(neighbor), neighbor))
            self.frontierSet.add(neighbor)

    def is_in_frontier(self, node):
        return node in self.frontierSet

    def min_cost_to_goal(self, node):
        return abs(node[0]-self.goal[0])+abs(node[1]-self.goal[1])


    def get_path(self, state):
        path = [state]
        while path[0] != self.start:
            path.insert(0, self.came_from[path[0]])
        return path



def RunBiStar(maze, start, goal):
    startTree = AStar(maze, start, goal)
    goalTree = AStar(maze, goal, start)

    # returns the best
    def bestPossibleCost():
        bestStart = startTree.frontier.get()
        startTree.frontier.put(bestStart)
        bestGoal = goalTree.frontier.get()
        goalTree.frontier.put(bestGoal)

        return bestStart[0] + bestGoal[0]


    def makePath(midpoint):
        return startTree.get_path(midpoint) + reversed(goalTree.get_path(midpoint))[1:]


    currentGoalCost = float('inf')
    currentGoal = None

    treePairs = [(startTree, goalTree), (goalTree, startTree)]
    iterationCount = 0

    while currentGoalCost < bestPossibleCost() and len(fromTree.frontier) + len(toTree.frontier) > 0:
        trees = treePairs[iterationCount % 2]

        fromTree = trees[0]
        toTree = trees[1]

        newState = fromTree.grow()
        if newState in toTree.frontier:
            newGoalCost = fromTree.cost_to[newState] + toTree.cost_to[newState]
            if newGoalCost < currentGoalCost:
                currentGoal = newState
                currentGoalCost = newGoalCost

        ++iterationCount
    return makePath(currentGoal), startTree.closed_set + goalTree.closed_set




class labyrinthe(list):
    def __init__(self, size):
        self.size = size
        labx, laby = size
        lx, ly = labx + 2, laby + 2
        ref = [1, lx, -1, -lx]
        l = [[random.randrange(lx + 1, lx * (ly - 1), lx) + random.randint(0, labx), random.choice(ref)]]
        L = list((0, 0, 0, 0) * lx + ((0, 0, 0, 0) + (1, 1, 1, 1) * labx + (0, 0, 0, 0)) * laby + (0, 0, 0, 0) * lx)
        L = [L[i:i + 4] for i in range(0, lx * ly * 4, 4)]
        self.extend(L)
        while l:
            for i in l:
                a = sum(i)
                b = (1 if abs(i[1]) == lx else lx) * random.choice((1, -1))
                if all(self[a]):
                    c = ref.index(i[1])
                    self[i[0]][c] = 0
                    i[0] = a
                    self[i[0]][c - 2] = 0
                    if not random.randint(0, 1): l.append([i[0], b])
                    if not random.randint(0, 3): l.append([i[0], -b])
                else:
                    if all(self[i[0] + b]): l.append([i[0], b])
                    if all(self[i[0] - b]): l.append([i[0], -b])
                    l.remove(i)
        del (self[:lx])
        del (self[-lx:])
        del (self[::lx])
        del (self[lx - 2::lx - 1])


    def get_path(self, start, exit):
        return RunBiStar(self, start, exit)


    def left_hand_rule(self, start, exit):
        pos = start
        d = 1
        path = [pos]
        ref = [1, self.size[0], -1, -self.size[0]]
        while pos != exit:
            if self[pos][ref.index(d) - 1] == 0: d = ref[ref.index(d) - 1]
            if self[pos][ref.index(d)] == 0:
                pos = pos + d
                path.append(pos)
                i = path.index(pos)
                if i != len(path) - 1:
                    del (path[i:-1])
            else:
                d = ref[ref.index(d) - 3]
        return path


    def get_neighbors(self, pos):
        moves = [1, self.size[0], -1, -self.size[0]]
        neighbors = []
        for i in range(len(moves)):
            if self[pos][i] == 0:
                neighbors.append(pos + moves[i])
        return neighbors


    def get_image_and_rects(self, cellulesize, wallcolor=(0, 0, 0), celcolor=(255, 255, 255)):
        x, y = cellulesize
        image = Surface((x * (self.size[0]), y * self.size[1]))
        image.fill(wallcolor)
        rects = []
        for e, i in enumerate(self):
            rects.append(image.fill(celcolor, (
                e % (self.size[0]) * cellulesize[0] + 1 - (not i[2]),
                e / (self.size[0]) * cellulesize[1] + 1 - (not i[3]),
                cellulesize[0] - 2 + (not i[2]) + (not i[0]), cellulesize[1] - 2 + (not i[1]) + (not i[3]))))
        return image, rects

# ****************************************************************************************
#****************************************************************************************
if __name__ == '__main__':
    me = Surface((5, 5))
    me.fill(0xff0000)
    L = labyrinthe((50, 50))
    labx, laby = 50, 50
    screen = display.set_mode((L.size[0] * 10, L.size[1] * 10))
    image, rectslist = L.get_image_and_rects((10, 10), wallcolor=0, celcolor=0xffffff)
    screen.blit(image, (0, 0))
    start = random.randrange(len(L))
    exit = random.randrange(len(L))
    screen.fill(0x00ff00, rectslist[exit])
    screen.blit(me, rectslist[start])
    display.flip()
    while event.wait().type != QUIT:
        screen.fill(-1, rectslist[start])
        if key.get_pressed()[K_RIGHT] and not L[start][0]:
            start += 1
        if key.get_pressed()[K_LEFT] and not L[start][2]:
            start += -1
        if key.get_pressed()[K_UP] and not L[start][3]:
            start += -L.size[1]
        if key.get_pressed()[K_DOWN] and not L[start][1]:
            start += L.size[0]
        screen.fill(0xff0000, rectslist[start])
        display.flip()
        if start == exit: print 'YOU WIN'; break
        if key.get_pressed()[K_ESCAPE]:
            for i in L.get_path(start, exit)[1:-1]:
                screen.fill(0x0000ff, rectslist[i])
                display.update(rectslist[i])
                time.wait(20)
