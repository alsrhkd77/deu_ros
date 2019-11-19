#! /usr/bin/env python

base = [-7.92, 5.3] #start position
goal = [7.95, -5.15]    #finish position
path = {0:[None, base, None, 90.0]}   #postnode, position(x,y), direction('left'/'right'/'both')
#path = {0:[None, goal, None]}   #postnode, position(x,y), direction('left'/'right'/'both')
stk = [int(0)]   #path stack

def temp():
    global path, stk
    count = 0

    count += 1
    path[count] = [count - 1, goal, 'both']
    stk.append(count)

    count += 1
    path[count] = [count -1, [8.04, -3.41], 'both']
    stk.append(count)

    count += 1
    path[count] = [count - 1, [4.40, -3.56], 'both']
    stk.append(count)

    count += 1
    path[count] = [count - 1, [4.54, -1.95], 'both']
    stk.append(count)

    count += 1
    path[count] = [count - 1, [6.15, -1.84], 'both']
    stk.append(count)

    count += 1
    path[count] = [count - 1, [6.12, 1.90], 'both']
    stk.append(count)

    count += 1
    path[count] = [count - 1, [7.87, 1.65], 'both']
    stk.append(count)

    count += 1
    path[count] = [count - 1, [7.90, 3.83], 'both']
    stk.append(count)

    count += 1
    path[count] = [count - 1, [2.58, 3.79], 'both']
    stk.append(count)

    count += 1
    path[count] = [count - 1, [2.76, 1.54], 'both']
    stk.append(count)

    count += 1
    path[count] = [count - 1, [0.83, 1.54], 'both']
    stk.append(count)

    count += 1
    path[count] = [count - 1, [0.90, 3.65], 'both']
    stk.append(count)

    count += 1
    path[count] = [count - 1, [-2.70, 3.69], 'both']
    stk.append(count)

    count += 1
    path[count] = [count - 1, [-2.70, 1.65], 'both']
    stk.append(count)

    count += 1
    path[count] = [count - 1, [-0.66, 1.72], 'both']
    stk.append(count)

    count += 1
    path[count] = [count - 1, [-0.56, -1.88], 'both']
    stk.append(count)

    count += 1
    path[count] = [count - 1, [-2.88, -1.80], 'both']
    stk.append(count)

    count += 1
    path[count] = [count - 1, [-2.70, -3.66], 'both']
    stk.append(count)

    count += 1
    path[count] = [count - 1, [-7.95, -3.70], 'both']
    stk.append(count)

    count += 1
    path[count] = [count - 1, [-7.99, -1.80], 'both']
    stk.append(count)

    count += 1
    path[count] = [count - 1, [-6.20, -1.73], 'both']
    stk.append(count)

    count += 1
    path[count] = [count - 1, [-6.09, -0.20], 'both']
    stk.append(count)

    count += 1
    path[count] = [count - 1, [-7.84, -0.34], 'both']
    stk.append(count)

    #count += 1
    #path[count] = [count - 1, base, 'both']
    #stk.append(count)
