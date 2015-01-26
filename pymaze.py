from pygame import*
quit
import random


from Queue import PriorityQueue




# (x,y)

class AStar:
	def __init__(self, start, goal):
		self.start = start
		self.goal = goal

		self.frontier = PriorityQueue()
		self.frontierSet = set() # same contents as frontier, but stored as a set for O(1) lookup

		self.closed_set = set()

		self.cost_so_far = {start: 0}
		self.came_from = {start: None}


		def grow(self, maze):
			# remove and return the lowest-cost item from the frontier
			current = self.frontier.get()

			if current == self.goal:
				return current

			self.closed_set.add(current)

			for neighbor in maze.get_neighbors(current):
				tentative_cost_so_far = cost_so_far[current] + 1
				# TODO

			# TODO: Alex


def BiStar(maze, start, goal):
	# TODO: Justin








class labyrinthe(list):
	''
	def __init__(self,size):
		self.size = size
		labx,laby = size
		lx,ly = labx+2,laby+2
		ref = [1,lx,-1,-lx]
		l=[[random.randrange(lx+1,lx*(ly-1),lx)+random.randint(0,labx),random.choice(ref)]]
		L = list((0,0,0,0)*lx+((0,0,0,0)+(1,1,1,1)*labx+(0,0,0,0))*laby+(0,0,0,0)*lx)
		L = [L[i:i+4] for i in range(0,lx*ly*4,4)]
		self.extend(L)
		while l:
			for i in l:
				a = sum(i)
				b  = (1 if abs(i[1])==lx else lx)*random.choice((1,-1))
				if all(self[a]):
					c = ref.index(i[1])
					self[i[0]][c] = 0
					i[0] = a
					self[i[0]][c-2] = 0
					if not random.randint(0,1): l.append([i[0],b])
					if not random.randint(0,3): l.append([i[0],-b])
				else :
					if all(self[i[0]+b]): l.append([i[0],b])
					if all(self[i[0]-b]): l.append([i[0],-b])
					l.remove(i)
		del(self[:lx])
		del(self[-lx:])
		del(self[::lx])
		del(self[lx-2::lx-1])

		
	def get_path(self,start,exit):
		pos = start
		d = 1
		path = [pos]
		ref = [1,self.size[0],-1,-self.size[0]]
		while pos != exit:
			if self[pos][ref.index(d)-1] == 0: d = ref[ref.index(d)-1]
			if self[pos][ref.index(d)] == 0:
				pos = pos+d
				path.append(pos)
				i = path.index(pos)
				if i != len(path)-1:
					del(path[i:-1])			
			else: d = ref[ref.index(d)-3]
		return path


	def get_neighbors(self, pos):
		moves = [1, self.size[0],-1,-self.size[0]]
		neighbors = []
		for i in range(len(moves)):
			if self[pos][i] == 0:
				neighbors.append(pos+moves[i])
		return neighbors


	def get_image_and_rects(self,cellulesize,wallcolor=(0,0,0),celcolor=(255,255,255)):
		x,y = cellulesize
		image = Surface((x*(self.size[0]),y*self.size[1]))
		image.fill(wallcolor)
		rects = []
		for e,i in enumerate(self):
			rects.append(image.fill(celcolor,(e%(self.size[0])*cellulesize[0]+1-(not i[2]),e/(self.size[0])*cellulesize[1]+1-(not i[3]),cellulesize[0]-2+(not i[2])+(not i[0]),cellulesize[1]-2+(not i[1])+(not i[3]))))
		return image,rects

#****************************************************************************************
#****************************************************************************************
if __name__ == '__main__':
	me = Surface((5,5))
	me.fill(0xff0000)
	L = labyrinthe((50,50))
	labx,laby = 50,50
	screen = display.set_mode((L.size[0]*10,L.size[1]*10))
	image,rectslist = L.get_image_and_rects((10,10),wallcolor=0,celcolor=0xffffff)
	screen.blit(image,(0,0))
	start = random.randrange(len(L))
	exit = random.randrange(len(L))
	screen.fill(0x00ff00,rectslist[exit])
	screen.blit(me,rectslist[start])
	display.flip()
	while event.wait().type != QUIT:
		screen.fill(-1,rectslist[start])
		if key.get_pressed()[K_RIGHT] and not L[start][0]:
			start += 1
		if key.get_pressed()[K_LEFT] and not L[start][2]:
			start += -1
		if key.get_pressed()[K_UP] and not L[start][3]:
			start += -L.size[1]
		if key.get_pressed()[K_DOWN] and not L[start][1]:
			start += L.size[0]
		screen.fill(0xff0000,rectslist[start])
		display.flip()
		if start == exit : print 'YOU WIN'; break
		if key.get_pressed()[K_ESCAPE]:
			for i in L.get_path(start,exit)[1:-1]:
				screen.fill(0x0000ff,rectslist[i])
				display.update(rectslist[i])
				time.wait(20)
