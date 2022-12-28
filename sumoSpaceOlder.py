import traci
import os
import math
import numpy as np
import random
from decimal import Decimal
from sympy import *
import threading
import time
from scipy.optimize import fsolve


sumoBinary = "C:\\Program Files (x86)\\Eclipse\\Sumo\\bin\\sumo-gui.exe"
sumoCmd = [sumoBinary, "-c",
           "C:\\Users\\user\\Desktop\\sumo\\laneChange\\space-older\\myConfig.sumocfg"]
traci.start(sumoCmd)

# traci start
# 變數宣告
step = 0
allAre1 = 0

# 開檔案
f = 0
f2 = 0
f3 = 0
f4 = 0

# 停止換車道參數
stopChangeFlag = 0

# 車輛長度與數量
carLength = 4
carNumber = 10

# 車間距離變數
space = []
# 車間距離數量
matrixLength = 0
spaceMatrix = []
fleetMatrix = []

# 計算空間排序變數
miniIndex = []
miniIndexSpace = []
miniIndexSpaceCount = []
mappingStartEndSorting = []

# 車輛與障礙物之距離
carCrash = []
carCrashTime = []

# 安全距離(舊)
safeSpce = 3

# 重新計算Flag
restartFlag = 0

# 新安全距離(跟車風險為1時)
safeDistace = []
selectdone = []

# 車隊車速
fleetSpeed = []

# 計算所需空間(舊)
changeNeedSpace = np.zeros(10)

# 滿足空間
satisSpace = []
satisTmp = []

# 滿足空間號碼
satisSpaceNumber = []
newSatisSpaceNumber = []

# 空間開始到結束
startEndSpace = []
mappingStartEnd = []

# 計算風險安全距離
x = Symbol('x')

# 車輛座標位置
fleetPosition = []
otherPosition = []

# 車輛id
fleetLandID = []
otherLandID = []

# 是否完成切換車道
changeFlag = []

# 開始切換速度的時間
changeSpeedTime = []

minSatiSpace = 0
minSatirange = ""

# 計算單個空間可以放幾台車
spaceCarCount = []

# 車子分配到那一個空間
carAssignSpace = []

# 群組矩陣
groupArray = []
groupChangeArray = []
groupChangeTime = []
groupChangeSpeedUp = []
groupChangeFlag = []
groupSafty = []

# 變化車道減速度
decArray = []

# 停止計算車隊減速進行換車道
decStopCal = 0
decStopFlag = 0
recoverSpeedFlag = 0

# 複製空間矩陣
spaceCopy = []
maxSpace = 0
maxIndex = 0
secondMax = 0
secondMaxIndex = 0

h1 = 0
firstGroupSpaceDec = 0
secondGroupSpaceDec = 0
h2 = 0
firstGroupSpaceSafety = 0
secondGroupSpaceSafety = 0
firstGroupNumber = 0

groupSpaceArray = []
groupStartChangeArray = []




# inital function start
# 寫檔初始化


def writeInit():
    global f, f2, f3, f4
    os.remove("output.txt")
    print("File removed successfully")
    path = 'output.txt'
    f = open(path, 'a')

    os.remove("output2.txt")
    print("File removed successfully")
    path = 'output2.txt'
    f2 = open(path, 'a')

    os.remove("output3.txt")
    print("File removed successfully")
    path = 'output3.txt'
    f3 = open(path, 'a')

    os.remove("output4.txt")
    print("File removed successfully")
    path = 'output4.txt'
    f4 = open(path, 'a')


# 初始化車子分配空間
def initCarAssign():
    global carAssignSpace
    for i in range(10):
        carAssignSpace.append(-1)
        carCrash.append(0)
        carCrashTime.append(0)
        decArray.append(-500)


# 初始化車輛位置(不只位置)
def initPosition():
    global fleetPosition, otherPosition, fleetLandID, otherLandID, saftyDistace, fleetSpeed
    for i in range(11):
        fleetPosition.append(0)
        otherPosition.append(0)
        fleetLandID.append(-1)
        otherLandID.append(-1)
        safeDistace.append(-1)
        fleetSpeed.append(0)
        selectdone.append(0)

# old 換車道所需空間


def staticChangeSpace():
    for i in range(carNumber):
        changeNeedSpace[i] = (i + 1) * (carLength + safeSpce)


# 初始化changeFlag
def changeInit():
    for i in range(11):
        changeFlag.append(0)

# 車隊車速初始化


def speedInit():
    # for i in range(11):
    #     vehString3 = "veh"+str(i)
    #     traci.vehicle.setSpeed(vehString3, 20)
    traci.vehicle.setSpeed("stop", 0)


# 變換車道時間初始化
def changeTime():
    for i in range(11):
        changeSpeedTime.append(-1)

# initial function end

# function start

# 計算跟車風險(跟車加速度)


def calFollowSpeed(nowSpeed, frontSpeed, distance, notice, brakeMax):
    follow = (nowSpeed**2 * brakeMax) / (frontSpeed**2 +
                                         2 * brakeMax * (distance - nowSpeed * notice))
    return follow


# 計算跟車安全距離
def calRiskBySpeed(nowSpeed, frontSpeed, maxBrake, notice, expect):
    solve_value = solve([((nowSpeed**2 * maxBrake)/(frontSpeed**2 +
                        2*maxBrake*(x - nowSpeed * notice)) / maxBrake) - expect], [x])
    return solve_value

# 計算緊急煞車滑行距離


def calSlideDistace(speed, maxBrake, time):
    slideDistace = speed * time - 0.5 * maxBrake * (time**2)
    return slideDistace

# 取得車輛行駛車道


def getAllLaneID():
    for i in range(11):
        veh = "veh" + str(i)
        no = "no" + str(i)
        fleetLandID[i] = traci.vehicle.getLaneIndex(veh)
        otherLandID[i] = traci.vehicle.getLaneIndex(no)

# 取得車輛位置


def getAllPosition():
    for i in range(11):
        veh = "veh" + str(i)
        no = "no" + str(i)
        fleetPosition[i] = traci.vehicle.getPosition(veh)[0]
        otherPosition[i] = traci.vehicle.getPosition(no)[0]


# 計算最近跟車車輛
def calClosestCar(now, index):
    # 當前車輛車道位置
    landID = traci.vehicle.getLaneIndex(now)
    # 當前車輛位置
    nowPosition = traci.vehicle.getPosition(now)[0]
    min = -1
    minIndex = 0
    # 是不是車隊車車輛
    fleetOrNot = -1
    for i in range(11):
        if landID == fleetLandID[i] and fleetPosition[index] < fleetPosition[i] and i != index:
            if min == -1:
                min = fleetPosition[i] - fleetPosition[index]
                minIndex = i
                fleetOrNot = 1
                # print("case 1: "+str(min))
            else:
                if min > fleetPosition[i] - fleetPosition[index]:
                    min = fleetPosition[i] - fleetPosition[index]
                    minIndex = i
                    fleetOrNot = 1
                    # print("case 2: "+str(min))
    for i in range(11):
        if landID == otherLandID[i] and fleetPosition[index] < otherPosition[i] and i != index:
            if min == -1:
                min = otherPosition[i] - fleetPosition[index]
                minIndex = i
                fleetOrNot = 0
                # print("case 3: "+str(min))
            else:
                if min > otherPosition[i] - fleetPosition[index]:
                    min = otherPosition[i] - fleetPosition[index]
                    minIndex = i
                    fleetOrNot = 0
                    # print("case 4: "+str(min))
    return [fleetOrNot, minIndex]

# 設定非車隊換車道模式


def noNoChange():
    for i in range(12):
        no = "no" + str(i)
        traci.vehicle.setLaneChangeMode(no, 0b000000000000)


# 設定車隊換車道模式
def fleetNoChange():
    for i in range(11):
        veh = "veh" + str(i)
        traci.vehicle.setLaneChangeMode(veh, 0b000000000000)
    traci.vehicle.setLaneChangeMode("stop", 0b000000000000)


# 設定車隊車輛速度模式
def speedModeInit(speedMode):
    # speedMode = 0
    for i in range(11):
        veh = "veh" + str(i)
        traci.vehicle.setSpeedMode(veh, speedMode)


# 空間放入矩陣
def getSpace():
    for i in range(10):
        no = "no"+str(i)
        no2 = "no"+str(i+1)
        gap = traci.vehicle.getPosition(
            no)[0] - traci.vehicle.getPosition(no2)[0]
        space[i] = gap - 4


# 取得車隊長度
def getFleetLength():

    # fleetLength = 0
    # for i in range(0, 10):
    #     veh = "veh" + str(i)
    #     veh2 = "veh" + str(i+1)
    #     frontSpeed = traci.vehicle.getSpeed(veh)
    #     nowSpeed = traci.vehicle.getSpeed(veh2)
    #     notice = 0.1
    #     solve_value = calRiskBySpeed(nowSpeed, frontSpeed, 8, notice, 1)
    #     if ":" in str(solve_value):
    #         characters = "{,x:}"
    #         tempString = str(solve_value)
    #     for j in range(len(characters)):
    #         tempString = tempString.replace(characters[j], "")
    #     saftyDistace = float(tempString)
    #     fleetLength = fleetLength + saftyDistace
    #     fleetLength = fleetLength + 4
    # fleetLast = fleetLength + safeSpce
    # return fleetLength
    veh = "veh" + str(0)
    veh2 = "veh" + str(10)
    fleetLength = traci.vehicle.getPosition(
        veh)[0] - traci.vehicle.getPosition(veh2)[0] + 4
    print(fleetLength)
    return fleetLength


# 計算取得安全距離
def calculateSafeDistance():
    global saftyDistace
    for i in range(0, 10):
        veh = "veh"+str(i)
        veh2 = "veh"+str(i+1)
        nowSpeed = traci.vehicle.getSpeed(veh2)
        frontSpeed = traci.vehicle.getSpeed(veh)

        if nowSpeed != fleetSpeed[i+1] or frontSpeed != fleetSpeed[i]:
            fleetSpeed[i+1] = nowSpeed
            fleetSpeed[i] = frontSpeed
            # print("nowSpeed: "+str(nowSpeed))
            # print("frontSpeed: "+str(frontSpeed))

            solve_value = calRiskBySpeed(nowSpeed, frontSpeed, 8, 0.1, 1)
            if ":" in str(solve_value):
                characters = "{,x:}"
                tempString = str(solve_value)
                for j in range(len(characters)):
                    tempString = tempString.replace(characters[j], "")
                safty = float(tempString)
                if safty < 1:
                    safeDistace[i] = 1
                else:
                    safeDistace[i] = safty

    # print(safeDistace)


# 移動車隊車輛位置
def moveVehiclePosition():
    temp1 = 0
    temp2 = 0
    for i in range(11):
        veh = "veh"+str(i)
        if i == 0:
            temp1 = traci.vehicle.getPosition(veh)[0]
            temp2 = traci.vehicle.getPosition(veh)[1]
            temp1 = temp1 + 500
            traci.vehicle.moveToXY(veh, "1to2", 0, temp1, temp2)
        elif i == 10:
            temp1 = temp1 - 100
            traci.vehicle.moveToXY(veh, "1to2", 0, temp1, temp2)
        else:
            temp1 = temp1 - 5
            traci.vehicle.moveToXY(veh, "1to2", 0, temp1, temp2)


# 填滿矩陣空間
def calMatrixSpace():
    global matrixLength
    for i in range(matrixLength):
        for j in range(matrixLength):
            if i == j:
                spaceMatrix[i, j] = space[i]
                startEndSpace[i][j] = str(i) + "-" + str(j)
            else:
                if i > j:
                    start = j
                    end = i
                    tmp = 0
                    for k in range(start, end+1):
                        tmp = tmp + space[k]
                    spaceMatrix[i, j] = tmp
                    startEndSpace[i][j] = str(start) + "-" + str(end)
                    tmp = 0
                else:
                    start = i
                    end = j
                    tmp = 0
                    for k in range(start, end+1):
                        tmp = tmp + space[k]
                    spaceMatrix[i, j] = tmp
                    startEndSpace[i][j] = str(start) + "-" + str(end)
                    tmp = 0
    # print(spaceMatrix)


# 空間符合車隊長度
def calFitFleetSpace(fleetLast):
    print("fleetLast: "+str(fleetLast))
    frontSpeed = traci.vehicle.getSpeed("no0")
    solve_value = calRiskBySpeed(frontSpeed, frontSpeed, 8, 1.5, 1)
    if ":" in str(solve_value):
        characters = "{,x:}"
        tempString = str(solve_value)
        for j in range(len(characters)):
            tempString = tempString.replace(characters[j], "")
        safty = float(tempString)

    for i in range(matrixLength):
        for j in range(matrixLength):
            if spaceMatrix[i, j] > fleetLast + 0 * (carLength + safeSpce):
                # print(spaceMatrix[i, j])
                start, end = startEndSpace[i][j].split("-")
                if(otherPosition[int(start)] - safty > fleetPosition[0]):
                    satisSpace.append(5000)
                else:
                    satisSpace.append(spaceMatrix[i, j])
                # satisSpace.append(spaceMatrix[i, j])
                satisSpaceNumber.append(abs(i - j) + 1)
                satisTmp.append(abs(i - j) + 1)
                newSatisSpaceNumber.append(abs(i - j) + 1)
                mappingStartEnd.append(startEndSpace[i][j])
    print(satisSpace)
    # print("\n")

# 空間排序


def sortingSpace():
    global minSatiSpace, minSatirange
    no = "no0"
    frontSpeed = traci.vehicle.getSpeed(no)
    safty = 0
    solve_value = calRiskBySpeed(frontSpeed, frontSpeed, 8, 1.5, 1)
    if ":" in str(solve_value):
        characters = "{,x:}"
        tempString = str(solve_value)
        for j in range(len(characters)):
            tempString = tempString.replace(characters[j], "")
        safty = float(tempString)

    minSatisSpaceNumber = min(newSatisSpaceNumber)
    print("minSatisSpaceNumber:"+str(minSatisSpaceNumber))
    for i in range(10):
        no = "no"+str(i)
        if fleetPosition[0] > otherPosition[i] - safty:
            minSatisSpaceNumber = i
            # print("sdfdsdddsfsd:"+str(i))
            break
    minSatirange = str(minSatisSpaceNumber)+"-"+str(minSatisSpaceNumber)
    minSatiSpace = satisSpace[minSatisSpaceNumber]

    # minSatiSpace = 0
    # minSatirange = ""
    # for i in range(len(newSatisSpaceNumber)):
    #     if newSatisSpaceNumber[i] == minSatisSpaceNumber:
    #         if minSatiSpace == 0:
    #             minSatiSpace = satisSpace[i]
    #             minSatirange = mappingStartEnd[i]
    #         elif minSatiSpace >satisSpace[i]:
    #             minSatiSpace = satisSpace[i]
    #             minSatirange = mappingStartEnd[i]

    # print(minSatiSpace)
    # print(minSatirange)

    # while min(satisTmp) != 10000:
    #     for i in range(len(satisTmp)):
    #         if satisSpaceNumber[i] == min(satisTmp):
    #             miniIndex.append(i)
    #             miniIndexSpace.append(satisSpace[i])
    #             mappingStartEndSorting.append(mappingStartEnd[i])
    #             satisTmp[i] = 10000
    # newSatisSpaceNumber.sort()

    # print(miniIndexSpace)
    # print("\n")
    # print(mappingStartEndSorting)
    # print("\n")
    # print(newSatisSpaceNumber)

    # print(len(satisSpace))

    satisSpace.clear()
    satisSpaceNumber.clear()
    newSatisSpaceNumber.clear()
    mappingStartEnd.clear()
    satisTmp.clear()


# 選擇最小的空間
def sortingMiniChoose():
    global restartFlag, minSatirange, spaceCarCount, matrixLength
    chooseSpace = []
    spaceCarCount.clear()
    start, end = minSatirange.split("-")
    startInt = 0
    endInt = 0
    now = 0
    groupArray.clear()
    groupChangeArray.clear()
    groupChangeTime.clear()
    groupChangeSpeedUp.clear()
    groupChangeFlag.clear()
    groupSafty.clear()
    frontSpeed = traci.vehicle.getSpeed("no0")
    solve_value = calRiskBySpeed(frontSpeed, frontSpeed, 8, 1.5, 1)
    if ":" in str(solve_value):
        characters = "{,x:}"
        tempString = str(solve_value)
        for j in range(len(characters)):
            tempString = tempString.replace(characters[j], "")
        safty = float(tempString)
    # print("matrixLength: "+str(matrixLength))

    if restartFlag == 0:
        start = int(start)
        end = int(end)
        print(minSatirange)
    elif restartFlag == 1:
        if int(end) + 1 > matrixLength - 1:
            print(minSatirange)
            start = int(start) - 1
            end = int(end)
            minSatirange = str(start) + "-" + str(end)
            print(minSatirange)
        else:
            print(minSatirange)
            start = int(start)
            end = int(end) + 1
            minSatirange = str(start) + "-" + str(end)
            print(minSatirange)
    restartFlag = 0
    if start >= 0:
        for i in range(start, end+1):
            chooseSpace.append(space[i])
        print("asdasdasd: "+str(chooseSpace))

        # fleetMatrix
        for i in range(len(chooseSpace)):
            if countAll(spaceCarCount) > carNumber:
                break
            for j in range(10):
                if start == end:
                    restartFlag = 1
                    break
                elif j+1 < 10 and chooseSpace[i] >= changeNeedSpace[j]+(safty*2) - safeSpce and chooseSpace[i] < changeNeedSpace[j+1]+(safty*2) - safeSpce:
                    print("number: "+str(j))
                    print(changeNeedSpace[j]+(safty*2))
                    print("number: "+str(j+1))
                    print(changeNeedSpace[j+1]+(safty*2))
                    if countAll(spaceCarCount)+(j+1) > carNumber:
                        tmp = carNumber-(countAll(spaceCarCount))
                        spaceCarCount.append(tmp)
                        if now == 0:
                            groupArray.append(0)
                            groupChangeArray.append(0)
                            groupChangeTime.append(0)
                            groupChangeSpeedUp.append(0)
                            groupChangeFlag.append(0)
                            groupSafty.append(0)
                            now = now + tmp
                        else:
                            groupArray.append(now)
                            groupChangeArray.append(0)
                            groupChangeSpeedUp.append(0)
                            groupChangeTime.append(0)
                            groupChangeFlag.append(0)
                            groupSafty.append(0)
                            now = now + tmp
                        if tmp != 0:
                            chooseSpace[i] = chooseSpace[i] - \
                                changeNeedSpace[tmp]-(safty*2)
                        break
                    else:
                        spaceCarCount.append(j+1)
                        if now == 0:
                            groupArray.append(0)
                            groupChangeArray.append(0)
                            groupChangeTime.append(0)
                            groupChangeSpeedUp.append(0)
                            groupChangeFlag.append(0)
                            groupSafty.append(0)
                            now = now + j+1
                        else:
                            groupArray.append(now)
                            groupChangeArray.append(0)
                            groupChangeTime.append(0)
                            groupChangeSpeedUp.append(0)
                            groupChangeFlag.append(0)
                            groupSafty.append(0)
                            now = now + j+1

                        chooseSpace[i] = chooseSpace[i] - \
                            changeNeedSpace[j]-(safty*2)
                        break
        print("dsdsdsssqq:"+str(countAll(spaceCarCount)))
        print("dsdsdsssqq:"+str(carNumber))
        if(countAll(spaceCarCount) < carNumber or restartFlag == 1):
            print("countAll: "+str(countAll(spaceCarCount)))
            print("carNumber: "+str(carNumber))
            # print(carNumber)

            restartFlag = 1
            sortingMiniChoose()
        else:
            print(minSatirange)
            print(countAll(spaceCarCount))
            print(spaceCarCount)
            print("\n")
            print(groupArray)

    # print("dfssfdsf"+str(chooseSpace)

# 計算array累加


def countAll(list):
    tmp = 0
    for i in range(len(list)):
        tmp = tmp + list[i]
    return tmp


# 分配車輛到指定空間
def carFittingSpace():
    start, end = minSatirange.split("-")
    start = int(start)
    end = int(end)
    count = 0
    whichCount = 0
    for i in range(len(spaceCarCount)):
        while count != spaceCarCount[i]:
            carAssignSpace[whichCount] = i + start
            whichCount = whichCount + 1
            count = count + 1
        count = 0
    print(carAssignSpace)


# 換車道分配
def distributeSpace():
    global decStopCal, decStopFlag
    global maxSpace, maxIndex, secondMax, secondMaxIndex, h1,h2
    global firstGroupSpaceDec, firstGroupSpaceSafety,secondGroupSpaceDec,secondGroupSpaceSafety,firstGroupNumber
    global groupSpaceArray,groupStartChangeArray

    groupSpaceArray.append(maxIndex + 1)
    groupSpaceArray.append(secondMaxIndex + 1)
    print("groupSpaceArray: "+str(groupSpaceArray))

    groupStartChangeArray.append(maxIndex)
    groupStartChangeArray.append(secondMaxIndex)



    groupArray.append(0)
    secondGroupStart = 0 + firstGroupNumber
    groupArray.append(secondGroupStart)
    print("test:"+str(groupArray))

    groupChangeArray.append(0)
    groupChangeTime.append(0)
    groupChangeSpeedUp.append(0)
    groupChangeFlag.append(0)
    groupSafty.append(0)

    groupChangeArray.append(0)
    groupChangeTime.append(0)
    groupChangeSpeedUp.append(0)
    groupChangeFlag.append(0)
    groupSafty.append(0)

    stopPosition = traci.vehicle.getPosition("stop")[0]
    timeDec = (stopPosition - 4 - fleetPosition[0]) / fleetSpeed[0]
    timeDec = timeDec - 3
    flagMinus = 0

    locationH1 = traci.vehicle.getPosition("no"+str(maxIndex))[0] - 4
    locationH2 = traci.vehicle.getPosition("no"+str(secondMaxIndex))[0] - 4

    newSpaceSpeed1 = traci.vehicle.getSpeed("no"+str(maxIndex))-((-firstGroupSpaceDec)*timeDec)
    print("newSpaceSpeed1: "+str(newSpaceSpeed1))
    newSpaceSpeed2 = traci.vehicle.getSpeed("no"+str(secondMaxIndex))-((-secondGroupSpaceDec)*timeDec)
    print("newSpaceSpeed2: "+str(newSpaceSpeed2))


    groupOneAnswer = calACCSpeedDiff(locationH1,fleetPosition[0],fleetSpeed[0],newSpaceSpeed1,timeDec,1.5,firstGroupSpaceDec)
    print("groupOneAnswer :"+str(groupOneAnswer))

    groupTwoAnswer = calACCSpeedDiff(locationH2,fleetPosition[secondGroupStart],fleetSpeed[secondGroupStart],newSpaceSpeed2,timeDec,1.5,secondGroupSpaceDec)
    print("groupTwoAnswer :"+str(groupTwoAnswer))

    if groupTwoAnswer[0][2] < 0:
        groupTwoAnswer = calACCSpeedDiff2(locationH2,fleetPosition[secondGroupStart],fleetSpeed[secondGroupStart],newSpaceSpeed2,timeDec,1.5,secondGroupSpaceDec)
        flagMinus = 1
    print("groupTwoAnswer :"+str(groupTwoAnswer))

    decArray[0] = groupOneAnswer[0][0]
    if decArray[0] > 8:
        decArray[0] = 8
    groupSafty[0] = groupOneAnswer[0][2]
    if flagMinus == 1:
        if ":" in str(groupTwoAnswer):
            characters = "{:x newSpeed eSafe}"
            tempString = str(groupTwoAnswer)
        for j in range(len(characters)):
            tempString = tempString.replace(characters[j], "")
        key = tempString
        splitString = key.split(',')
        if float(splitString[0]) > 8:
            decArray[1] = 8
        else:
            decArray[1] = float(splitString[0])
        groupSafty[1] = float(splitString[2])
    else:
        if float(groupTwoAnswer[0][0]) > 8:
            decArray[1] = 8
        else:
            decArray[1] = groupTwoAnswer[0][0]
        groupSafty[1] = groupTwoAnswer[0][2]

    print("decArray[0]: "+str(decArray[0]))
    print("decArray[1]: "+str(decArray[1]))

    decStopFlag = 1

    
    # for i in range(len(groupArray)):
    #     nowSpeed = traci.vehicle.getSpeed("veh"+str(groupArray[i]))
    #     frontSpeed = traci.vehicle.getSpeed("no"+str(start + i))
    #     solve_value = calRiskBySpeed(frontSpeed, frontSpeed, 8, 1.5, 1)
    #     if ":" in str(solve_value):
    #         characters = "{,x:}"
    #         tempString = str(solve_value)
    #         for j in range(len(characters)):
    #             tempString = tempString.replace(characters[j], "")
    #         safty = float(tempString)
    #         groupSafty[i] = safty
    #     timeDec = (stopPosition - 4 -
    #                fleetPosition[groupArray[i]]) / fleetSpeed[groupArray[i]]
    #     timeDec = timeDec - 3
    #     print("time: "+str(timeDec))



        # timeDec2 = fleetSpeed[groupArray[i]] / 8
        # timeDec = timeDec * 0.5 + timeDec2 * 0.5
        # print("time: "+str(timeDec))

    #     if i == 0:
    #         print("otherPosition: "+str(otherPosition[start + i]))
    #         print("fleetPosition: "+str(fleetPosition[groupArray[i]]))
    #         print("groupSafty: "+str(groupSafty[i]))
    #         # print("otherPosition: "+str(otherPosition[start + i]-fleetPosition[groupArray[i]] +4 +groupSafty[i]))

    #     if timeDec != 0:
    #         # dec = ((fleetPosition[groupArray[i]] - otherPosition[start + i] - safty) / ((fleetSpeed[groupArray[i]] - frontSpeed)* timeDec)) * 2 / timeDec
    #         # dec = 2 * ((fleetPosition[groupArray[i]] - otherPosition[start + i] - safty) - ((fleetSpeed[groupArray[i]] - frontSpeed)* timeDec)) /  (timeDec ** 2)
    #         dec = -((otherPosition[start + i]-fleetPosition[groupArray[i]] + 4 - groupSafty[i]) - (
    #             (fleetSpeed[groupArray[i]] - frontSpeed) * timeDec)) / (timeDec ** 2 * 0.5)
    #         print("dec: "+str(dec))
    #         if dec > 8:
    #             dec = 8
    #         decArray[i] = dec
    #         if dec > 0:
    #             if i == len(groupArray) - 1:
    #                 for j in range(groupArray[i], 10):
    #                     veh = "veh" + str(j)
    #                     if traci.vehicle.getSpeed(veh) - dec/100 > 0:
    #                         traci.vehicle.setSpeed(
    #                             veh, traci.vehicle.getSpeed(veh) - dec/100)
    #                     else:
    #                         traci.vehicle.setSpeed(veh, 0)

    #                     # traci.vehicle.setApparentDecel(veh, dec)
    #                     # print(str(j)+": "+str(traci.vehicle.getSpeed(veh)))
    #             else:
    #                 for j in range(groupArray[i], groupArray[i+1]):
    #                     veh = "veh" + str(j)
    #                     if traci.vehicle.getSpeed(veh) - dec / 100 > 0:
    #                         traci.vehicle.setSpeed(
    #                             veh, traci.vehicle.getSpeed(veh) - dec/100)
    #                     else:
    #                         traci.vehicle.setSpeed(veh, 0)
    #                     # traci.vehicle.setApparentDecel(veh, dec)
    #                     # print(str(j)+": "+str(traci.vehicle.getSpeed(veh)))

    # for i in range(len(groupArray)):
    #     if decArray[i] > 0:
    #         decStopCal = decStopCal + 1
    #     if decStopCal == len(groupArray):
    #         decStopFlag = 1
    # decStopCal = 0


# 持續減速
def countiDecGroup():
    global decStopCal, decStopFlag
    global maxSpace, maxIndex, secondMax, secondMaxIndex, h1,h2
    global firstGroupSpaceDec, firstGroupSpaceSafety,secondGroupSpaceDec,secondGroupSpaceSafety,firstGroupNumber
    global groupSpaceArray

    
    # print(minSatirange)

    for i in range(len(groupSpaceArray)):
        if groupChangeArray[i] == 0:
            if i == 0:
                for j in range(groupSpaceArray[i],groupSpaceArray[i+1]):
                    no = "no" + str(j)
                    if traci.vehicle.getSpeed(no) - (-firstGroupSpaceDec /100) > 0:
                        traci.vehicle.setSpeed(no, traci.vehicle.getSpeed(no) - (-firstGroupSpaceDec /100))
                    else:
                        traci.vehicle.setSpeed(no, 0)
                    print("no"+str(j)+" :"+str(traci.vehicle.getSpeed(no)))
            else:
                for j in range(groupSpaceArray[i],11):
                    no = "no" + str(j)
                    if traci.vehicle.getSpeed(no) - (-secondGroupSpaceDec /100) > 0:
                        traci.vehicle.setSpeed(no, traci.vehicle.getSpeed(no) - (-secondGroupSpaceDec /100))
                    else:
                        traci.vehicle.setSpeed(no, 0)
                    print("no"+str(j)+" :"+str(traci.vehicle.getSpeed(no)))
                         

    for i in range(len(groupArray)):
        if groupChangeArray[i] == 0:
            if i == len(groupArray) - 1:
                # print("dec "+str(i)+": "+str(decArray[i]))
                for j in range(groupArray[i], 10):
                    veh = "veh" + str(j)
                    if traci.vehicle.getSpeed(veh) - decArray[i]/100 > 0:
                        traci.vehicle.setSpeed(
                            veh, traci.vehicle.getSpeed(veh) - decArray[i]/100)
                    else:
                        traci.vehicle.setSpeed(veh, 0)
                    # traci.vehicle.setApparentDecel(veh, dec)
                    print(str(j)+": "+str(traci.vehicle.getSpeed(veh)))
            else:
                # print("dec "+str(i)+": "+str(decArray[i]))
                for j in range(groupArray[i], groupArray[i+1]):
                    veh = "veh" + str(j)
                    if traci.vehicle.getSpeed(veh) - decArray[i]/100 > 0:
                        traci.vehicle.setSpeed(
                            veh, traci.vehicle.getSpeed(veh) - decArray[i]/100)
                    else:
                        traci.vehicle.setSpeed(veh, 0)
                    # traci.vehicle.setApparentDecel(veh, dec)
                    print(str(j)+": "+str(traci.vehicle.getSpeed(veh)))

# 檢查空間與車隊距離
def spaceFleetClose():
    global step, groupStartChangeArray

    # groupStartChangeArray.append(maxIndex)
    # groupStartChangeArray.append(secondMaxIndex)

    # start, end = minSatirange.split("-")
    # start = int(start)
    # end = int(end)
    # print("start: :"+minSatirange)
    frontSpeed = 0
    for i in range(len(groupArray)):
        # print(groupArray)
        # print(groupSafty[i])
        print("safty: "+str(groupSafty[i]))
        # nowSpeed2 = traci.vehicle.getSpeed("veh"+str(groupArray[i]))
        # frontSpeed2 = traci.vehicle.getSpeed("no"+str(start + i))
        # if groupSafty[i] == 0:
        #     nowSpeed = traci.vehicle.getSpeed("veh"+str(groupArray[i]))
        #     frontSpeed = traci.vehicle.getSpeed("no"+str(start + i))
        #     solve_value = calRiskBySpeed(frontSpeed, frontSpeed, 8, 1.5, 1)
        #     if ":" in str(solve_value):
        #         characters = "{,x:}"
        #         tempString = str(solve_value)
        #         for j in range(len(characters)):
        #             tempString = tempString.replace(characters[j], "")
        #         safty = float(tempString)
        #         groupSafty[i] = safty

        # print(abs(otherPosition[start+i] - fleetPosition[groupArray[i]]-groupSafty[i])<1)
        # print(abs(otherPosition[start+i] - fleetPosition[groupArray[i]]-groupSafty[i])<1)
        # print(abs(otherPosition[start+i] - fleetPosition[groupArray[i]]-groupSafty[i])<1)
        # print(abs(otherPosition[start+i] - fleetPosition[groupArray[i]]-groupSafty[i])<1)

        # if fleetPosition[groupArray[i]]>0:
        #     nowSpeed*1.5


        print (abs(otherPosition[groupStartChangeArray[i]] - 4 - fleetPosition[groupArray[i]]-groupSafty[i]))
        if i == len(groupArray) - 1:
            # if fleetPosition[groupArray[i]] < otherPosition[start + i] and abs(otherPosition[start+i]- 4 - fleetPosition[groupArray[i]]-groupSafty[i]) < (frontSpeed2-nowSpeed2)*5+((frontSpeed2-nowSpeed2)/6)*(frontSpeed2-nowSpeed2) and groupChangeArray[i] == 0 and groupSafty[i] > 0:
            # if abs(otherPosition[start+i] - 4 - fleetPosition[groupArray[i]]-groupSafty[i]) < (frontSpeed2-nowSpeed2)*2 and groupChangeArray[i] == 0 and groupSafty[i] > 0:
            if abs(otherPosition[groupStartChangeArray[i]] - 4 - fleetPosition[groupArray[i]]-groupSafty[i]) < 4 and groupChangeArray[i] == 0 and groupSafty[i] > 0:
                # print("safty: "+str(groupSafty[i]))
                groupChangeTime[i] = step
                for j in range(groupArray[i], 10):
                    veh = "veh" + str(j)
                    traci.vehicle.changeLane(veh, 1, 300)
                    # traci.vehicle.setSpeed(veh, 10)
                    groupChangeArray[i] = 1
        else:
            # abs(otherPosition[start+i] - fleetPosition[groupArray[i]]) < 3

            # print(str(i)+" :"+str((otherPosition[start+i] - 4 - fleetPosition[groupArray[i]] - groupSafty[i])))

            # if fleetPosition[groupArray[i]] < otherPosition[start+i] and abs(otherPosition[start+i] - 4 - fleetPosition[groupArray[i]]-groupSafty[i]) < (frontSpeed2-nowSpeed2)*5+((frontSpeed2-nowSpeed2)/6)*(frontSpeed2-nowSpeed2) and groupChangeArray[i] == 0 and groupSafty[i] > 0:
            # if abs(otherPosition[start+i] - 4 - fleetPosition[groupArray[i]] - groupSafty[i]) < (frontSpeed2-nowSpeed2)*2 and groupChangeArray[i] == 0 and groupSafty[i] > 0:
            if abs(otherPosition[groupStartChangeArray[i]] - 4 - fleetPosition[groupArray[i]] - groupSafty[i]) < 4 and groupChangeArray[i] == 0 and groupSafty[i] > 0:
                groupChangeTime[i] = step
                for j in range(groupArray[i], groupArray[i+1]):
                    veh = "veh" + str(j)
                    traci.vehicle.changeLane(veh, 1, 300)
                    # traci.vehicle.setSpeed(veh, 10)
                    groupChangeArray[i] = 1

# 換完車道回復車速
def laneChangeSpeedRecover():
    global step, recoverSpeedFlag

    global decStopCal, decStopFlag
    global maxSpace, maxIndex, secondMax, secondMaxIndex, h1,h2
    global firstGroupSpaceDec, firstGroupSpaceSafety,secondGroupSpaceDec,secondGroupSpaceSafety,firstGroupNumber
    global groupSpaceArray

    # start, end = minSatirange.split("-")
    # start = int(start)
    # end = int(end)

    
    for i in range(len(groupArray)):
        if i == len(groupArray) - 1 and groupChangeArray[i] == 1:
            if groupChangeArray[i] == 1 and step >= groupChangeTime[i] + 150 and groupChangeFlag[i] == 0:
                # if  groupChangeSpeedUp[i] == 0:
                #     frontSpeed = traci.vehicle.getSpeed("no"+str(start+i))
                #     groupChangeSpeedUp[i] = otherPosition[start+i] - fleetPosition[groupArray[i]] / (frontSpeed - fleetSpeed[groupArray[i]])
                # if otherPosition[start+i] - fleetPosition[groupArray[i]]:
                frontSpeed = traci.vehicle.getSpeed("no"+str(groupSpaceArray[i]))
                for j in range(groupArray[i], 10):
                    print("speed"+str(j)+": "+str(fleetSpeed[j]))
                    veh = "veh" + str(j)
                    # traci.vehicle.setSpeed(veh, frontSpeed)
                    if abs(fleetSpeed[j] - frontSpeed) < 0.05:
                        groupChangeFlag[i] = 1
                    else:
                        traci.vehicle.setSpeed(veh, fleetSpeed[j] + 6 / 100)
                # groupChangeFlag[i] = 1
        elif groupChangeArray[i] == 1:
            if groupChangeArray[i] == 1 and step >= groupChangeTime[i] + 150 and groupChangeFlag[i] == 0:
                # if  groupChangeSpeedUp[i] == 0:
                #     frontSpeed = traci.vehicle.getSpeed("no"+str(start+i))
                #     groupChangeSpeedUp[i] = otherPosition[start+i] - fleetPosition[groupArray[i]] / (frontSpeed - fleetSpeed[groupArray[i]])
                frontSpeed = traci.vehicle.getSpeed("no"+str(groupSpaceArray[i]))
                for j in range(groupArray[i], groupArray[i+1]):
                    print("speed"+str(j)+": "+str(fleetSpeed[j]))
                    veh = "veh" + str(j)
                    # traci.vehicle.setSpeed(veh, frontSpeed)
                    if abs(fleetSpeed[j] - frontSpeed) < 0.05:
                        groupChangeFlag[i] = 1
                    else:
                        traci.vehicle.setSpeed(veh, fleetSpeed[j] + 6 / 100)

                # groupChangeFlag[i] = 1


# 計算車隊車輛與障礙物之距離
def calCarAndCrash():
    global carCrash, fleetPosition, carCrashTime
    crashPosition = traci.vehicle.getPosition("stop")[0]
    for i in range(10):
        if fleetSpeed[i] > 0:
            carCrash[i] = crashPosition - fleetPosition[i] - carLength
            carCrashTime[i] = carCrash[i] / fleetSpeed[i]


# 選擇最大和次大的空間
def chooseTwoMaxSpace():
    global spaceCopy, maxSpace, maxIndex, secondMax, secondMaxIndex
    spaceCopy = space.copy()
    # print(space)
    # spaceCopy = space.copy()
    for i in range(len(spaceCopy)):
        if otherPosition[i] > fleetPosition[0]:
            spaceCopy[i] = 0
    maxSpace = max(spaceCopy)
    maxIndex = spaceCopy.index(maxSpace)
    spaceCopy[maxIndex] = 0
    secondMax = max(spaceCopy)
    secondMaxIndex = spaceCopy.index(secondMax)

    print("Max Space Lenghth: "+str(maxSpace))
    print("Max Space Index: "+str(maxIndex))

    print("Second Max Space Lenghth: "+str(secondMax))
    print("Second Max Space Index: "+str(secondMaxIndex))


# 計算第一空間所需要之空間
def firstSpaceNeeded():
    global maxSpace, maxIndex, secondMax, secondMaxIndex, h1
    if otherPosition[maxIndex] > otherPosition[secondMaxIndex]:
        h1 = otherPosition[maxIndex] - otherPosition[secondMaxIndex] - 4
    else:
        h1 =  otherPosition[secondMaxIndex]-otherPosition[maxIndex] - 4
    # h1 = otherPosition[maxIndex] - otherPosition[secondMaxIndex] - 4
    print("otherPosition[maxIndex]:"+str(otherPosition[maxIndex]))
    print("otherPosition[secondMaxIndex]:"+str(otherPosition[secondMaxIndex]))
    print("h1: "+str(h1))


# 計算所需要的空間
def calculateAllSpaceNeeded():
    global maxSpace, maxIndex, secondMax, secondMaxIndex, h1,h2
    global firstGroupSpaceDec, firstGroupSpaceSafety,secondGroupSpaceDec,secondGroupSpaceSafety,firstGroupNumber

    ai = Symbol('ai')
    aj = Symbol('aj')
    h2 = Symbol('h2')
    t = Symbol('t')
    e = Symbol('e')

    eSafe1 = Symbol('eSafe1')
    eSafe2 = Symbol('eSafe2')


    speedI = traci.vehicle.getSpeed("no"+str(maxIndex))
    speedJ = traci.vehicle.getSpeed("no"+str(secondMaxIndex))

    print("speedI: "+str(speedI))
    print("speedJ: "+str(speedJ))

    speedI_front = traci.vehicle.getSpeed("no"+str(maxIndex-1))
    speedJ_front = traci.vehicle.getSpeed("no"+str(secondMaxIndex-1))


    di = space[maxIndex]
    dj = space[secondMaxIndex]

    print("di: "+str(di))
    print("dj: "+str(dj))


    carLenghth = 4
    fleetCarNumber = 10
    stopPosition = traci.vehicle.getPosition("stop")[0]


    timeDec = (stopPosition - 4 -fleetPosition[0]) / fleetSpeed[0]
    timeDec = timeDec - 3
    print("timeDec: "+str(timeDec))
    t = timeDec


    solve_value = solve([
                        h1-di+eSafe1*2+((speedI_front-speedI)*t+0.5*ai*t**2), 
                        h2-dj+h1+eSafe2*2+((speedJ_front-speedJ)*t+0.5*(ai+aj)*t**2), 
                        # speedI+ai*t-(speedJ+aj*t), 
                        h1-(e*((((speedI+ai*t)**2-(speedI+ai*t)**2)/(2*8)+(speedI+ai*t)*0.1)+carLenghth)+eSafe1*2), 
                        h2-((fleetCarNumber-e)*((((speedJ+aj*t)**2-(speedJ+aj*t)**2)/(2*8)+(speedJ+aj*t)*0.1)+carLenghth)+eSafe2*2),
                        (((speedI+ai*t)**2 * 8) /((speedI+ai*t)**2 +2*8*(eSafe1 - (speedI+ai*t) * 1.5)) / 8) - 1,
                        (((speedJ+aj*t)**2 * 8) /((speedJ+aj*t)**2 +2*8*(eSafe2 - (speedJ+aj*t) * 1.5)) / 8) - 1,
                        ], [ai, aj, h2,e,eSafe1,eSafe2])

    print(solve_value)
    newE = int(solve_value[0][3])
    firstGroupNumber = newE
    print(newE)
    solve_value = solve([
                        # h1-di+eSafe*2+((speedI_front-speedI)*t+0.5*ai*t**2), 
                        h2-dj+h1+eSafe2*2+((speedJ_front-speedJ)*t+0.5*(ai+aj)*t**2), 
                        # speedI+ai*t-(speedJ+aj*t), 
                        (-(-di+eSafe1*2+((speedI_front-speedI)*t+0.5*ai*t**2)))-(newE*((((speedI+ai*t)**2-(speedI+ai*t)**2)/(2*8)+(speedI+ai*t)*0.1)+carLenghth)+eSafe1*2), 
                        h2-((fleetCarNumber-newE)*((((speedJ+aj*t)**2-(speedJ+aj*t)**2)/(2*8)+(speedJ+aj*t)*0.1)+carLenghth)+eSafe2*2),
                        (((speedI+ai*t)**2 * 8) /((speedI+ai*t)**2 +2*8*(eSafe1 - (speedI+ai*t) * 1.5)) / 8) - 1,
                        (((speedJ+aj*t)**2 * 8) /((speedJ+aj*t)**2 +2*8*(eSafe2 - (speedJ+aj*t) * 1.5)) / 8) - 1,
                        ], [ai, aj, h2,eSafe1,eSafe2])
    print(solve_value)

    if ":" in str(solve_value):
        characters = "{:ai aj h2 eSafe1 eSafe2}"
        tempString = str(solve_value)
    for j in range(len(characters)):
        tempString = tempString.replace(characters[j], "")
    key = tempString
    splitString = key.split(',')
    print(splitString)
    # dec = float(splitString[0])
    firstGroupSpaceDec = float(splitString[0])
    secondGroupSpaceDec = float(splitString[1])
    h2 = float(splitString[2])
    firstGroupSpaceSafety = float(splitString[3])
    secondGroupSpaceSafety = float(splitString[4])

    return splitString
    



    # solve_value = solve([
    #                     h1-di+eSafe*2-(speedI*t+0.5*ai*t**2), 
    #                     h2-dj+h1+eSafe*2-(speedJ*t+0.5*aj*t**2), 
    #                     speedI+ai*t-(speedJ+aj*t), 
    #                     h1-(e*((((speedI+ai*t)**2-(speedI+ai*t)**2)/(2*8)+(speedI+ai*t)*0.1)+carLenghth)+eSafe*2), 
    #                     h2-((fleetCarNumber-e)*((((speedJ+aj*t)**2-(speedJ+aj*t)**2)/(2*8)+(speedJ+aj*t)*0.1)+carLenghth)+eSafe*2),
    #                     (((speedI+ai*t)**2 * 8) /((speedI+ai*t)**2 +2*8*(eSafe - (speedI+ai*t) * 1.5)) / 8) - 1,
    #                     ], [ai, aj, h2, t,e,eSafe])


    # solve_value = solve([h1-di+eSafe*2-(speedI*t+0.5*ai*t*t), 
    #                     h2-dj+h1+eSafe*2-(speedJ*t+0.5*aj*t*t), 
    #                     speedI+ai*t-(speedJ+aj*t), 
    #                     h1-(e*((speedI+ai*t)*((speedI+ai*t)/8)+(0.5*(-8)*((speedI+ai*t)/8)**2)+carLenghth)+eSafe*2), 
    #                     h2-((fleetCarNumber-e)*((speedJ+aj*t)*((speedJ+aj*t)/8)+(0.5*(-8)*((speedJ+aj*t)/8)**2)+carLenghth)+eSafe*2),
    #                     (((speedI+ai*t)**2 * 8) /((speedI+ai*t)**2 +2*8*(eSafe - (speedI+ai*t) * 1.5)) / 8) - 1,
    #                     ], [ai, aj, t, h2, e,eSafe])

    # solve_value = solve([h1-di-(speedI*t+0.5*ai*t*t), 
    #                         h2-dj+h1-(speedJ*t+0.5*aj*t*t), 
    #                         speedI+ai*t-(speedJ+aj*t), 
    #                         h1-(e*((speedI+ai*t)*((speedI+ai*t)/8)+(0.5*(-8)*((speedI+ai*t)/8)**2)+carLenghth)), 
    #                         h2-((fleetCarNumber-e)*((speedJ+aj*t)*((speedJ+aj*t)/8)+(0.5*(-8)*((speedJ+aj*t)/8)**2)+carLenghth))], [ai, aj, t, h2, e])                        
    

    # ai, aj, time, h2, e = unsolvedValue[0], unsolvedValue[1], unsolvedValue[2], unsolvedValue[3], unsolvedValue[4]
    # return [
    #     h1-di-(speedI*time + 0.5*ai*(time**2)),
    #     h2-dj+h1-(speedJ*time + 0.5*aj*(time**2)),
    #     speedI-ai*time-(speedJ-aj*time),
    #     h1-(e*((speedI-ai*time)*((speedI-ai*time)/8) -(0.5*8*(((speedI-ai*time)/8))**2)+carLenghth)),
    #     h2 - ((fleetCarNumber-e)*((speedJ-aj*time)*((speedJ-aj*time)/8) -(0.5*8*(((speedJ-aj*time)/8)**2)+carLenghth)))
    # ]


#新計算減速度(安全距離) 
def calACCSpeedDiff(locationX,groupPosition,nowSpeed, frontSpeed,t,notice,spaceDec):
    eSafe = Symbol('eSafe')
    newSpeed = Symbol('newSpeed')

    solve_value = solve([locationX - groupPosition - eSafe + (frontSpeed - newSpeed)* 1.5 -((nowSpeed-frontSpeed)*t-(0.5*(x-spaceDec)*(t**2))), 
                        newSpeed-(nowSpeed - x * t), 
                        # eSafe - (((newSpeed**2 - frontSpeed**2) /(2 * 8)) + newSpeed*1.5)
                        # eSafe - newSpeed * notice
                        ((newSpeed**2 * 8) /(frontSpeed**2 +2*8*(eSafe - newSpeed * notice)) / 8) - 1
                        ], [x, newSpeed, eSafe],)

    return solve_value

# 修正安全距離為負
def calACCSpeedDiff2(locationX,groupPosition,nowSpeed, frontSpeed,t,notice,spaceDec):
    eSafe = Symbol('eSafe')
    newSpeed = Symbol('newSpeed')
    solve_value = solve([locationX - groupPosition - eSafe + (frontSpeed - newSpeed)* 1.5 -((nowSpeed-frontSpeed)*t-(0.5*(x-spaceDec)*(t**2))), 
                        newSpeed-(nowSpeed - x * t), 
                        # eSafe - (((newSpeed**2 - frontSpeed**2) /(2 * 8)) + newSpeed*1.5)
                        eSafe - newSpeed * notice
                        # ((newSpeed**2 * 8) /(frontSpeed**2 +2*8*(eSafe - newSpeed * notice)) / 8) - 1
                        ], [x, newSpeed, eSafe],)

    return solve_value


# 風險檢測
def riskFollowing():
    # 檢查flag
    global allAre1
    allAre1 = 0

    # 判斷前方車輛的種類
    for i in range(1, 11):
        now = "veh"+str(i)
        front = "veh"+str(i-1)
        closeset = calClosestCar(now, i)
        if closeset[0] == 0:
            front = "no"+str(closeset[1])
        elif closeset[0] == 1:
            front = "veh"+str(closeset[1])
        if "no" in front:
            notice = 1.5
        elif "no" in now:
            notice = 1.5
        elif "veh" in now and "veh" in front:
            notice = 0.1

        # 取得車輛速度
        frontSpeed = traci.vehicle.getSpeed(front)
        nowSpeed = traci.vehicle.getSpeed(now)
        # 車與車之間x座標之距離
        gap = traci.vehicle.getPosition(
            front)[0] - traci.vehicle.getPosition(now)[0] - 4
        # 計算理當前理想的跟車速度
        acci = calFollowSpeed(nowSpeed, frontSpeed, gap, notice, 8)

        # 計算單前風險值
        followRisk = acci / 8

        # 風險值大於1之情況
        if abs(followRisk) >= 1:
            solve_value = calRiskBySpeed(nowSpeed, frontSpeed, 8, notice, 1)
            if ":" in str(solve_value):
                characters = "{,x:}"
                tempString = str(solve_value)
                for j in range(len(characters)):
                    tempString = tempString.replace(characters[j], "")
                saftyDistace = float(tempString)

            # if i == 9:
            #     print("\n"+now+" follwing: "+front)
            #     print("nowSpeed: "+str(nowSpeed))
            #     print("frontSpeed: "+str(frontSpeed))
            #     print("notice: "+str(notice))
            #     print("laneID: "+str(fleetLandID[i]))
            #     print("followRisk : " +str(followRisk))
            #     print("safe distace : " +str(saftyDistace))
            #     print("gap : " +str(gap))
            #     print("acci: "+ str(acci))
            #

            # if saftyDistace > gap:
            #     offset = saftyDistace - gap

            # 以最大度進行減速
            if saftyDistace > gap:
                oneSpeed = 8 / 100
                if traci.vehicle.getSpeed(now) - oneSpeed > 0:
                    newSpeed = traci.vehicle.getSpeed(now) - oneSpeed
                    traci.vehicle.setSpeed(now, newSpeed)
                    if i == 9:
                        print("oneSpeed : " + str(oneSpeed))
                        print("newSpeed : " + str(newSpeed))

            # # 計算合理的減速度
            # if saftyDistace > gap:
            #     crashTime = gap / (nowSpeed)
            #     distaceOffset = saftyDistace - gap
            #     if crashTime <= notice:
            #         distaceOffset10 = (distaceOffset / 8) / 10
            #     else:
            #         oneSecondPlus = distaceOffset / crashTime
            #         distaceOffset10 = (distaceOffset / oneSecondPlus) / 10
            #     # print("distaceOffset10: "+str(distaceOffset10 * 10))
            #     if traci.vehicle.getSpeed(now) - distaceOffset10 > 0:
            #         newSpeed = traci.vehicle.getSpeed(now) - distaceOffset10
            #         traci.vehicle.setSpeed(now, newSpeed)
            #         # print ("newSpeed: "+str(newSpeed))
            # elif abs(saftyDistace - gap) <= 0.1:
            #     return 0
            # elif saftyDistace < gap:
            #     distaceOffset = gap - saftyDistace
            #     distaceOffset10 = (distaceOffset / 6)
            #     newSpeed = traci.vehicle.getSpeed(now) + distaceOffset / 2
            #     traci.vehicle.setSpeed(now, newSpeed)
        else:
            allAre1 = allAre1 + 1
    return allAre1
# traci end

# normal python
# 空間初始化


def spaceInit():
    global matrixLength, spaceMatrix, startEndSpace, fleetMatrix
    for i in range(10):
        space.append(0)
    # 空間矩陣初始化
    matrixLength = np.size(space)
    spaceMatrix = np.empty((matrixLength, matrixLength))
    fleetMatrix = np.empty((matrixLength, matrixLength))
    startEndSpace = [["1" for _ in range(matrixLength)]
                     for _ in range(matrixLength)]


def fleetCalculatePosition():
    vehPosition = traci.vehicle.getPosition("veh0")[0]
    vehPosition2 = traci.vehicle.getPosition("veh10")[0]
    allLength = abs(vehPosition - vehPosition2)+carLength
    return allLength


# 車隊所需空間大小
def fleetCalculate():
    global matrixLength, fleetMatrix
    for i in range(matrixLength):
        for j in range(matrixLength):
            veh = "veh" + str(i)
            veh1 = "veh" + str(j)
            vehPosition = traci.vehicle.getPosition(veh)[0]
            vehPosition1 = traci.vehicle.getPosition(veh1)[0]
            if i == j:
                fleetMatrix[i, j] = carLength + safeDistace[i]

            elif i > j:
                start = j
                end = i
                tmp = 0
                for k in range(start, end+1):
                    tmp = tmp + carLength + safeDistace[k]
                fleetMatrix[i, j] = tmp

            elif i < j:
                start = i
                end = j
                tmp = 0
                for k in range(start, end+1):
                    tmp = tmp + carLength + safeDistace[k]
                fleetMatrix[i, j] = tmp
                # fleetMatrix[i, j] = int(abs(vehPosition - vehPosition1) + carLength + safeSpce)
    # print("qeeqeq")
    # print ('%1.2f' %(fleetMatrix[0, 9]))


# 開始模擬
def startSimulate():
    global step
    global f, f2, f3
    global stopChangeFlag
    global decStopCal, decStopFlag
    while step < 40000:
        traci.simulationStep()
        if step == 0:
            staticChangeSpace()
            initPosition()
            initCarAssign()
            changeInit()
            speedInit()
            changeTime()
            noNoChange()
            fleetNoChange()
            speedModeInit(0)
            writeInit()
            spaceInit()

        if step == 50:
            moveVehiclePosition()

        # if step == 100:
        #     global spaceCopy
        #     spaceCopy = space.copy()

        if step > 200:
            calculateSafeDistance()
            getAllPosition()
            calCarAndCrash()
            getAllLaneID()
            getSpace()
            calMatrixSpace()
            fleetCalculate()
            # calFitFleetSpace(fleetMatrix[0, 9])

            # # calFitFleetSpace(fleetCalculatePosition())

            # sortingSpace()
            if step == 201:
                chooseTwoMaxSpace()
                firstSpaceNeeded()
                calculateAllSpaceNeeded()
                # print(solved)

                # calFitFleetSpace(fleetMatrix[0, 9]+(0)*2)
                # sortingSpace()
                # sortingMiniChoose()

                # carFittingSpace()

            # chooseTwoMaxSpace()

            # print("veh0 - stop: "+str(traci.vehicle.getPosition("stop")
            #       [0]-traci.vehicle.getPosition("veh0")[0]))
            # print("veh0 - no6 :"+str(traci.vehicle.getPosition("veh0")
            #       [0]-traci.vehicle.getPosition("no6")[0]))
            # if  traci.vehicle.getPosition("stop")[0] - traci.vehicle.getPosition("veh0")[0]< 200 and decStopFlag == 0 :
            if decStopFlag == 0 and step >= 201:
                # print("fsdfsdf")
                distributeSpace()
            if decStopFlag == 1:
                countiDecGroup()
            #     print("speedOther: "+str(traci.vehicle.getSpeed("no0")))
            #     # spaceFleetClose()
            #     # laneChangeSpeedRecover()
            spaceFleetClose()
            laneChangeSpeedRecover()

            # riskFollowing()

        if step == 1:
            for i in range(11):
                vehString = "veh"+str(i)
                noString = "no"+str(i)
                traci.vehicle.setSpeed(vehString, 30)
                traci.vehicle.setSpeed(noString, 32)
                if i == 10:
                    traci.vehicle.setSpeed(vehString, 0)

            # for i in range(10):
            #     noString = "no"+str(i)
            #     traci.vehicle.setDecel(noString, 0)
            #     traci.vehicle.setEmergencyDecel(noString, 0)
            #     traci.vehicle.setApparentDecel(noString, 0)

        if step > -1:
            save = ""
        for i in range(11):
            vehString = "veh"+str(i)
            save = save + " " + str(traci.vehicle.getSpeed(vehString))
        f.write(str(step/100)+save+"\n")
        save = ""
        for i in range(10):
            vehString = "veh"+str(i)
            vehString2 = "veh"+str(i+1)
            gap = 0
            gap = abs(traci.vehicle.getPosition(vehString2)[
                      0]-traci.vehicle.getPosition(vehString)[0])
            save = save + " " + str(gap)
        f2.write(str(step/100)+save+"\n")
        save = ""
        for i in range(10):
            vehString = "veh"+str(i)
            gap = str(traci.vehicle.getPosition(vehString)[0])
            save = save + " " + str(gap)
        f3.write(str(step/100)+save+"\n")
        save = ""
        for i in range(10):
            vehString = "veh"+str(i)
            gap = str(traci.vehicle.getPosition(vehString)[
                      0])+" "+str(traci.vehicle.getPosition(vehString)[1])
            save = save + " " + str(gap)
        f4.write(str(step/100)+save+"\n")

        step += 1


if __name__ == "__main__":
    startSimulate()
    print("start")
