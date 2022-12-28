import traci
import os
import math
import numpy as np
import random
from decimal import Decimal
from sympy import *
import threading
import time

x = Symbol('x')


def calRiskBySpeed(nowSpeed, frontSpeed, maxBrake, notice, expect):
    solve_value = solve([((nowSpeed**2 * maxBrake)/(frontSpeed**2 +
                        2*maxBrake*(x - nowSpeed * notice)) / maxBrake) - expect], [x])
    return solve_value


def calACC(t):
    solve_value = solve([(460-500+30)+(0.5*x*(t**2))], [x])
    return solve_value


def calACCSpeedDiff(nowSpeed, frontSpeed,t):
    solve_value = solve([(-10-46.0)-(nowSpeed-frontSpeed)*t+(0.5*x*(t**2))], [x])
    return solve_value


def calculateAllSpaceNeeded():
    global maxSpace, maxIndex, secondMax, secondMaxIndex, h1

    ai = Symbol('ai')
    aj = Symbol('aj')

    h2 = Symbol('h2')

    t = Symbol('t')
    e = Symbol('e')

    eSafe = Symbol('eSafe')
    timeDec = 20
    # eSafe2 = Symbol('eSafe2')

    # newSpeed1 = Symbol('newSpeed1')
    # newSpeed2 = Symbol('newSpeed2')

    # a2 = Symbol('ai')
    # a2 = Symbol('aj')

    

    speedI = 30
    speedJ = 30
    di = 60
    dj = 55
    carLenghth = 4
    fleetCarNumber = 10


    solve_value = solve([
                    h1-di-(speedI*timeDec+0.5*ai*timeDec**2), 
                    h2-dj+h1-(speedJ*timeDec+0.5*aj*timeDec**2), 
                    # speedI+ai*timeDec-(speedJ+aj*timeDec), 
                    h1-(e*((((speedI+ai*timeDec)**2-(speedI+ai*timeDec)**2)/(2*8)+(speedI+ai*timeDec)*0.1)+carLenghth)+eSafe*2), 
                    h2-((fleetCarNumber-e)*((((speedJ+aj*timeDec)**2-(speedJ+aj*timeDec)**2)/(2*8)+(speedJ+aj*timeDec)*0.1)+carLenghth)+eSafe*2),
                    (((speedI+ai*timeDec)**2 * 8) /((speedI+ai*timeDec)**2 +2*8*(eSafe - (speedI+ai*timeDec) * 1.5)) / 8) - 1,
                    ], [ai, aj, h2, e,eSafe])

    # solve_value = solve([h1-di-(speedI*t+0.5*ai*t*t), 
    #                     h2-dj+h1-(speedJ*t+0.5*aj*t*t), 
    #                     speedI+ai*t-(speedJ+aj*t), 
    #                     h1-(e*((speedI+ai*t)*((speedI+ai*t)/8)+(0.5*(-8)*((speedI+ai*t)/8)**2)+carLenghth)), 
    #                     h2-((fleetCarNumber-e)*((speedJ+aj*t)*((speedJ+aj*t)/8)+(0.5*(-8)*((speedJ+aj*t)/8)**2)+carLenghth))], [ai, aj, t, h2, e])

    return solve_value
    print(solve_value)


# def calculateDecPosition(locationX,groupPosition,nowSpeed, frontSpeed,t,notice):
#     global maxSpace, maxIndex, secondMax, secondMaxIndex, h1

#     eSafe1 = Symbol('eSafe1')
#     eSafe2 = Symbol('eSafe2')

#     newSpeed1 = Symbol('newSpeed1')
#     newSpeed2 = Symbol('newSpeed2')

#     a2 = Symbol('ai')
#     a2 = Symbol('aj')

#     speedI = traci.vehicle.getSpeed("no"+str(maxIndex))
#     speedJ = traci.vehicle.getSpeed("no"+str(secondMaxIndex))
#     di = space[maxIndex]
#     dj = space[secondMaxIndex]
#     carLenghth = 4
#     fleetCarNumber = 10

#     solve_value = solve([h1-di-(speedI*t+0.5*ai*t*t), 
#                         h2-dj+h1-(speedJ*t+0.5*aj*t*t), 
#                         speedI+ai*t-(speedJ+aj*t), 
#                         h1-(e*((speedI+ai*t)*((speedI+ai*t)/8)+(0.5*(-8)*((speedI+ai*t)/8)**2)+carLenghth)), 
#                         h2-((fleetCarNumber-e)*((speedJ+aj*t)*((speedJ+aj*t)/8)+(0.5*(-8)*((speedJ+aj*t)/8)**2)+carLenghth))], [ai, aj, t, h2, e])

    return solve_value
    print(solve_value)


print(calRiskBySpeed(30,30,8,1.5,1))


# 3.2833999999852495

time = 100
nowSpeed = 20
frontSpeed = 12
a = calACCSpeedDiff(nowSpeed,frontSpeed,time)
if ":" in str(a):
    characters = "{x:}"
    tempString = str(a)
    for j in range(len(characters)):
        tempString = tempString.replace(characters[j], "")
    a = float(tempString)
ans = nowSpeed - a*time
print("dec is: "+str(a))
print(ans)



# dec = - ((450-500 +11.5) - ((15 - 20)* time)) /  (time ** 2 * 0.5)
# print(dec)


# a = calACC(time)
# if ":" in str(a):
#     characters = "{x:}"
#     tempString = str(a)
#     for j in range(len(characters)):
#         tempString = tempString.replace(characters[j], "")
#     a = float(tempString)
# ans = 20 - a*time
# print("dec is: "+str(a))
# print(ans)
