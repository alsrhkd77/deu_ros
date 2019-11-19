#! /usr/bin/env python

'''
This values can replace custom MSG
'''

speed = 0
turn = 0

def increaseSpeed():
    global speed
    speed += 0.1
    if speed > 1:
        speed = 1


def decreaseSpeed():
    global speed
    speed -= 0.1
    if (speed < -1):
        speed = -1


def increaseLeftTurn():
    global turn
    turn += 0.2
    if (turn > 2):
        turn = 2


def increaseRightTurn():
    global turn
    turn -= 0.2
    if (turn < -2):
        turn = -2


def forceStop():
    global speed, turn
    speed = 0
    turn = 0


def turnLeft():
    global speed, turn
    speed = 0
    turn = 2


def turnRight():
    global speed, turn
    speed = 0
    turn = -2