#! /usr/bin/env python

mode = 'Explorer'
base = [-7.92, 5.3] #start position
map_goal = [7.95, -5.15]    #finish position
goal = [map_goal[0] - base[0], map_goal[1] - base[1]]   #relative finish position
path = {0:[None, [0, 0], None, 90.0]}   #postnode, position(x,y), direction('left'/'right'/'both'), entry angle
stk = [int(0)]   # path stack
unused = [] # unused path stack