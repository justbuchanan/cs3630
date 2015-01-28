from pygame import *
import math
import random
from Queue import PriorityQueue
import Queue
import time



class AStar:
    def __init__(self, maze, start, goal):
        self.maze = maze
        self.start = start
        self.goal = goal

        self.cost_to = {start: 0}
        self.came_from = {start: None}

        self.closed_set = set()
        self.frontier = PriorityQueue()

        self.frontier.put((self.min_cost_to_goal(start), start))


    def grow(self):
        # remove and return the lowest-cost item from the frontier
        current = self.frontier.get_nowait()
        if current == self.goal:
            return current[1]

        self.closed_set.add(current[1])

        for neighbor in self.maze.get_neighbors(current[1]):
            if neighbor not in self.closed_set:
                self.cost_to[neighbor] = self.cost_to[current[1]] + 1
                self.came_from[neighbor] = current[1]
                self.frontier.put((self.cost_to[neighbor] + self.min_cost_to_goal(neighbor), neighbor))

        return current[1]


    def min_cost_to_goal(self, state):
        s = self.maze.coords_for_state(state)
        g = self.maze.coords_for_state(self.goal)

        return abs(s[0]-g[0]) + abs(s[1]-g[1])


    def get_path(self, state):
        path = [state]
        while path[0] != self.start:
            path.insert(0, self.came_from[path[0]])
        return path


def RunAStar(maze, start, goal):
    searchTree = AStar(maze, start, goal)
    thisnode = None

    while not searchTree.frontier.empty() and not thisnode == searchTree.goal:
        thisnode = searchTree.grow()
    if not thisnode == searchTree.goal:
        return None #unsolvable maze
    return (searchTree.get_path(searchTree.goal), searchTree.closed_set)


def RunBiStar(maze, start, goal):
    startTree = AStar(maze, start, goal)
    goalTree = AStar(maze, goal, start)

    # returns the best
    def bestPossibleCost():
        try:
            bestStart = startTree.frontier.get_nowait()
            startTree.frontier.put(bestStart)
            bestGoal = goalTree.frontier.get_nowait()
            goalTree.frontier.put(bestGoal)
        except Queue.Empty:
            return float('inf')

        # edited to reflect the priority of the nodes being estimated total cost
        # ie, action cost to node + estimated cost to goal
        return min(bestStart[0], bestGoal[0])


    def makePath(midpoint):
        return startTree.get_path(midpoint) + list(reversed(goalTree.get_path(midpoint)))[1:]


    currentGoalCost = float('inf')
    currentGoal = None

    treePairs = [(startTree, goalTree), (goalTree, startTree)]
    iterationCount = 0

    while currentGoalCost > bestPossibleCost() and (not startTree.frontier.empty() and not goalTree.frontier.empty()):
        trees = treePairs[iterationCount % 2]

        fromTree = trees[0]
        toTree = trees[1]

        newState = fromTree.grow()
        if newState in toTree.closed_set:
            newGoalCost = fromTree.cost_to[newState] + toTree.cost_to[newState]
            if newGoalCost < currentGoalCost:
                currentGoal = newState
                currentGoalCost = newGoalCost
        iterationCount += 1

    explored = startTree.closed_set.union(goalTree.closed_set)
    if currentGoal == None:
        #Unsolvable Maze Encountered
        return None

    return makePath(currentGoal), explored




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


    def coords_for_state(self, state):
        x = state % self.size[0]
        y = state / self.size[0]
        return x, y


    def get_path(self, start, exit):
        return (RunBiStar(self, start, exit), RunAStar(self, start, exit))


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
    L = labyrinthe((50, 50))
    start = random.randrange(len(L))
    exit = random.randrange(len(L))

    # draw solution found from get_path()
    results =  L.get_path(start, exit)
    for result in results:
        me = Surface((5, 5))
        me.fill(0xff0000)
        labx, laby = 50, 50
        screen = display.set_mode((L.size[0] * 10, L.size[1] * 10))
        image, rectslist = L.get_image_and_rects((10, 10), wallcolor=0, celcolor=0xffffff)
        screen.blit(image, (0, 0))
        path, explored = result
        for pt in explored:
            screen.fill(0xff00ff, rectslist[pt])
        for pt in path:
            screen.fill(0x00ffff, rectslist[pt])

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


def test(func, size=(50,50)):
    L = labyrinthe(size)
    start = random.randrange(len(L))
    exit = random.randrange(len(L))
    startTime = time.time()
    ans = func(L, start, exit)
    if ans == None:
        return test(func)
    path, explored = func(L, start, exit)
    elapsedTime = time.time() - startTime

    sxy = L.coords_for_state(start)
    gxy = L.coords_for_state(exit)
    euclideanDist = math.sqrt((sxy[0]-gxy[0])**2 + (sxy[1]-gxy[1])**2)

    if euclideanDist == 0:
        CR = len(path) - 1
    else:
        CR = (len(path) - 1) / euclideanDist

    return elapsedTime, CR, len(explored)


def testNTimes(func, n):
    times = list()
    CRs = list()
    explored = list()
    print "Running " + str(n) + " trials of " + str(func)
    trials = [test(func) for i in range(n)]
    i = 0
    for trial in trials:
        times.append(trial[0])
        CRs.append(trial[1])
        explored.append(trial[2])
    print "Average Time: " + str(sum(times)/len(times))
    print "Average CR: " + str(sum(CRs)/len(CRs))
    print "Average # of nodes explored: " + str(sum(explored)/len(explored))

def testAStar100():
    testNTimes(RunAStar, 100)

def testBiStar100():
    testNTimes(RunBiStar, 100)
