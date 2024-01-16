import random
from field.node import Node
from model.airframe import Airframe
from routing.travellingSalesmanProblem import TravellingSalesmanProblem
from routing import calcThreshold
from routing.singleDP import SingleDP
from routing.doubleDroneRouting import DoubleDR
from routing.vrp import VRP
from model.multicopter import Multi
from model.vtol import Vtol
from model.largeMulti import LargeMulti
from model.smollMulti import SmollMulti
from field.map import Map
#import openpyxl
from matplotlib import pyplot
import matplotlib as mpl
import matplotlib.cm as cm
import numpy as np
from routing.vrpState import VrpState
from routing.singleRouting import SingleRouting

# map criate
def main0(path,N):
    Map.criateMapFile(N,path)
    
# 機体が各ペイロード量で何分飛行できるのか
def calcFlightabelTime():
    drone1 = Vtol()
    for i in range(11):
        BC1 = drone1.consum_f(i/10)
        print(i/10,(100-drone1.consum_h(i/10))/BC1)
    
#mapのみ表示
def main02(path):
    map = Map(path)
    map.showMap()
    
#分析用
def main03(mapPath,drone):
    drone = drone
    routing = SingleDP(drone,mapPath)
    routing.criateTBobjectB()
    routing.printBestRouteObjectB()
    
    routing.plotAnalysis()
    routing.plotRouteFig()
    
#閾値を求める
def main04():
    m = Multi()
    v = Vtol()
    
    calcThreshold.calcThreshold(m,v)
    
# ルートプロット
def main05(nodeList):
    fig = pyplot.figure()
    ax = fig.add_subplot(111)

    ax.plot(*[0,0], 'o', color="blue") #  デポのプロット
    for p in nodeList: #  ノードのプロット
        ax.plot(*[p.x,p.y], 'o', color="red")
        ax.text(p.x, p.y,p.demand)
    
    for i in range(len(nodeList)-1): #  矢印のプロット
        fromNode = nodeList[i]
        toNode = nodeList[i+1]
        ax.annotate('', xy=[toNode.x,toNode.y], xytext=[fromNode.x,fromNode.y],
                    arrowprops=dict(shrink=0, width=1, headwidth=8, 
                                    headlength=10, connectionstyle='arc3',
                                    facecolor='gray', edgecolor='gray')
                    )

    pyplot.show()
    
# 広いマップ作成
def main06(path,N,r,min_p,max_p):
    Map.criateLargeMapFile(N,r,min_p,max_p,path)
    
# 制限の範囲内での1機体でのルーティング
def main1(mapPath):
    drone1 = Multi()
    drone2 = Vtol()
    routing1 = SingleDP(drone1,mapPath)
    routing2 = SingleDP(drone2,mapPath)

    routing1.criateTBobjectB()
    print("multi")
    routing1.printBestRouteObjectB()
    #print(routing1.goalFlag)
    #routing1.plotRouteFig()

    routing2.criateTBobjectB()
    print("vtol")
    routing2.printBestRouteObjectB()
    #print(routing2.goalFlag)
    #routing2.plotRouteFig()
    
    if routing1.goalFlag == 1 and routing2.goalFlag == 1 :
        ddr = DoubleDR(drone1,drone2,mapPath)
        ddr.flightDrone1List = routing1.bestRoute
        ddr.flightDrone2List = routing2.bestRoute
        ddr.plotFig()
    elif routing1.goalFlag == 1 and routing2.goalFlag == 0:
        routing1.plotRouteFig()
    elif routing1.goalFlag == 0 and routing2.goalFlag == 1:
        routing2.plotRouteFig()
    
# 機体数２で固定での厳密解求解プログラム
def main2(mapPath):
    drone1 = Multi()
    drone2 = Vtol()
    DDR1 = DoubleDR(drone1,drone1,mapPath)
    DDR1.findMinBC2flight()
    print(drone1.__class__.__name__,"(blue):",DDR1.flightDrone1List,"ft ",DDR1.drone1FT,"BC ",DDR1.drone1BC)
    print(drone1.__class__.__name__,"(green):",DDR1.flightDrone2List,"ft ",DDR1.drone2FT,"BC ",DDR1.drone2BC)
    #print(drone1.__class__.__name__,drone1.__class__.__name__,"BC ",DDR1.drone1BC+DDR1.drone2BC)
    #DDR1.plotFig() #  青矢印がdrone1, 緑矢印がdrone2
    print()
    
    DDR2 = DoubleDR(drone2,drone2,mapPath)
    DDR2.findMinBC2flight()
    print(drone2.__class__.__name__,"(blue):",DDR2.flightDrone1List,"ft ",DDR2.drone1FT,"BC ",DDR2.drone1BC)
    print(drone2.__class__.__name__,"(green):",DDR2.flightDrone2List,"ft ",DDR2.drone2FT,"BC ",DDR2.drone2BC)
    #print(drone2.__class__.__name__,drone2.__class__.__name__,"BC ",DDR2.drone1BC+DDR2.drone2BC)
    #DDR2.plotFig() #  青矢印がdrone1, 緑矢印がdrone2
    print()
    
    DDR3 = DoubleDR(drone1,drone2,mapPath)
    DDR3.findMinBC2flight()
    print(drone1.__class__.__name__,"(blue):",DDR3.flightDrone1List,"ft ",DDR3.drone1FT,"BC ",DDR3.drone1BC)
    print(drone2.__class__.__name__,"(green):",DDR3.flightDrone2List,"ft ",DDR3.drone2FT,"BC ",DDR3.drone2BC)
    #print(drone1.__class__.__name__,drone2.__class__.__name__,"BC ",DDR3.drone1BC+DDR3.drone2BC)
    #DDR3.plotFig() #  青矢印がdrone1, 緑矢印がdrone2
    print()
    
    if DDR1.drone1BC+DDR1.drone2BC < DDR2.drone1BC+DDR2.drone2BC and DDR1.drone1BC+DDR1.drone2BC < DDR3.drone1BC+DDR3.drone2BC:
        print("multi+multi")
    elif DDR2.drone1BC+DDR2.drone2BC < DDR1.drone1BC+DDR1.drone2BC and DDR2.drone1BC+DDR2.drone2BC < DDR3.drone1BC+DDR3.drone2BC:
        print("vtol+vtol")
    elif DDR3.drone1BC+DDR3.drone2BC < DDR1.drone1BC+DDR1.drone2BC and DDR3.drone1BC+DDR3.drone2BC < DDR2.drone1BC+DDR2.drone2BC:
        print("multi+vtol")
    
# simannelを利用したsingleRoutingを実行実験する
def main3(drone1,mapFilePath):
    map = Map(mapFilePath)
    state = map.nodeList
    state.append(Node(0,0,0,0))
    tsp = TravellingSalesmanProblem(state,drone1)
    state = tsp.anneal()
    print()
    #print("長さ",len(state[0]))
    #for node in state[0]:
    #    print(node.nodeNum)
    print(state[1])
    main05(state[0])
    
# 動的計画法のsingleRouting
def main4(mapFilePath):
    miniMap = Map(mapFilePath)
    drone =Multi()
    allCustomerNum = miniMap.CN
    tsp = SingleRouting(miniMap.customerList,drone,allCustomerNum+3)
    tsp.criateTBobjectB()
    tsp.searchBestRouteObjectB()
    for n in tsp.bestRoute:
        print(n.nodeNum,end=" , ")
    print()
    print("BC",tsp.BC,"FT",tsp.FT)
    
#3機以上のドローンで
def main5(mapFilePath,droneNum, droneList):
    map = Map(mapFilePath)
    customerList = map.customerList
    
    #初期解作成
    initial_state = VrpState(droneNum,droneList,map.CN)
    i = 0
    for c in customerList:
        initial_state.miniCustomerMap[i].append(c)
        i += 1
        if i == droneNum:
            i = 0
    
    #到達不可能の顧客をカウント
    impCustomer = 0
    for j in range(droneNum):
        initial_state.calcCost(j)
        if initial_state.cost_list[j][0] == Airframe() and initial_state.miniCustomerMap[j] != []:
            impCustomer += 1

    # 初期解表示
    for i in range(droneNum):
        print("[",end=" ")
        for n in initial_state.eachFlights[i]:
            print(n.nodeNum,end=", ")
        print("]",initial_state.cost_list[i][0].type,"FT",format(initial_state.cost_list[i][1],'.2f'),"BC",format(initial_state.cost_list[i][2],'.2f'),"payload",format(initial_state.cost_list[i][3],'.2f'))
    
    
    vrp = VRP(initial_state)

    state = vrp.anneal()
    print()
    f = open('data/result.txt','a')
    #f.write("drone type, distance, payload, BC"+"\n")
    
    """
    for i in range(droneNum):
        if len(state[0].eachFlights[i])==0:
            continue
        print("[",end=" ")
        for n in state[0].eachFlights[i]:
            print(n.nodeNum,end=", ")
        print("]",state[0].cost_list[i][0].type,"FT",format(state[0].cost_list[i][1],'.2f'),"BC",format(state[0].cost_list[i][2],'.2f'),"payload",format(state[0].cost_list[i][3],'.2f'))
    
    #state[0].plotRouteFig()
    """
    
    for i in range(droneNum):
        if len(state[0].eachFlights[i])==0:
            #usedDrone -= 1
            continue
        d = 0
        for j in range(len(state[0].eachFlights[i])-1):
            d += map.distance2(state[0].eachFlights[i][j],state[0].eachFlights[i][j+1])
        print(state[0].cost_list[i][0].type,"customer amount",len(state[0].eachFlights[i])-2,"payload",format(state[0].cost_list[i][3],'.2f'),"distance",format(d,'.2f'),"BC",format(state[0].cost_list[i][2],'.2f'))
        f.write(state[0].cost_list[i][0].type+","+str(format(d,'.2f'))+","+str(format(state[0].cost_list[i][3],'.2f'))+","+str(format(state[0].cost_list[i][2],'.2f'))+","+str(len(state[0].eachFlights[i])-2)+"\n")
        
    f.close
    
    sumDemand = map.calcSumDemand()
    avePayload = sumDemand/map.CN
    aveDepoDistance = map.calcDepoDistanceAve
    #TODO state[1]には実行不可能解のペナルティ付きBCも含まれるので不適切
    sumBC = state[0].calcSumBC()
    f_BC = open("data/__BC","a")
    f_BC.write(str(aveDepoDistance)+","+str(avePayload)+","+str(sumBC)+","+str(impCustomer)+",\n")
    f_BC.close
    

def plotResultFile(path):
    fig = pyplot.figure()
    ax = fig.add_subplot(111)
    
    f = open(path,'r')
    next(f) #  ファイルの2行目から読み込み
    
    SMultiP = []
    SMultiD = []
    SMultiC = []
    LMultiP = []
    LMultiD = []
    LMultiC = []
    vtolP = []
    vtolD = []
    vtolC = []
    while True: #  マップのファイルから顧客リストを作成
        flightStr = f.readline() #  ファイルから1行読む
        if flightStr == '': #  EOFになったら終了
            break
    
        flightResultList = flightStr.split(',')
        payload = float(flightResultList[1])
        distance = float(flightResultList[2])
        customerAmount = str(flightResultList[4])
    
        if flightResultList[0] == "S multi":
            SMultiP.append(payload)
            SMultiD.append(distance)
            SMultiC.append(customerAmount)
        elif flightResultList[0] == "L multi":
            LMultiP.append(payload)
            LMultiD.append(distance)
            LMultiC.append(customerAmount)
        elif flightResultList[0] == "vtol":
            vtolP.append(payload)
            vtolD.append(distance)
            vtolC.append(customerAmount)
        
    ax.plot(*[SMultiP,SMultiD], 'o', color="gold", label = "S multi")
    ax.plot(*[LMultiP,LMultiD], 'o', color="green", label = "L multi")
    ax.plot(*[vtolP,vtolD], 'o', color="blue", label = "vtol")
    ax.legend()
    
    """
    #顧客数をグラフにプロット
    for i in range(len(SMultiP)):
        ax.text(SMultiP[i],SMultiD[i],SMultiC[i])
    for j in range(len(LMultiP)):
        ax.text(LMultiP[j],LMultiD[j],LMultiC[j])
    for k in range(len(vtolP)):
        ax.text(vtolP[k],vtolD[k],vtolC[k])
    """
    
    #ax.set_xlim([0, 35])
    #ax.set_ylim([0, 7])
    ax.set_xlabel("flight distance(km)")
    ax.set_ylabel("payload(kg)")
    ax.grid(axis="both")
    
    pyplot.show()


def plotBCFile(path):
    fig = pyplot.figure()
    ax = fig.add_subplot(111)
    
    ax.set_xlabel("average distance between depo and customer (km)")
    ax.set_ylabel("average demand of customer (kg)")
    #ax.set_xlim([0, 120])
    #ax.set_ylim([0, 1.2])
    
    ax.grid(axis="both")
    
    f = open(path,'r')
    next(f) #  ファイルの2行目から読み込み
    
    while True: 
        BCStr = f.readline() #  ファイルから1行読む
        if BCStr == '': #  EOFになったら終了
            break
    
        BCList = BCStr.split(',')
        aveDis = float(BCList[0])
        avePay = float(BCList[1])
        BC = float(BCList[2]) #round(float(BCList[2]),2)
        impCostomer = int(BCList[3])
    
        mp = ax.scatter(aveDis,avePay,s=50,c=BC,cmap="rainbow")#vmin=0,vmax=100
        if impCostomer != 0:
            ax.text(aveDis,avePay,impCostomer)
        
    cbar = pyplot.colorbar(mp)
    #cbar.ax.set_ylim(0,100)
    #cbar.ax.yaxis.set_major_locator(mpl.ticker.MultipleLocator(10))
    
    pyplot.show()

    
def tryNtimes(N,droneList):
    
    for i in range(N):
        mapName = "data/map_r4_p0.4_"+str(i)+".txt"
        #ランダムでエリア半径と顧客数を作成
        #r = random.randint(5,18) # 半径18で最長が50.8km vtolの最大飛行距離が50km
        r=12
        #customer = random.randint(2,25)
        customer = 10
        main06(mapName,customer,r,p=1.5)
        #main5(mapName,droneNum=customer,droneList=droneList)
    
    #plotBCFile("data/multiUsage")
    #plotResultFile('data/result.txt')
        
        
if __name__ == "__main__":
    droneList = [Vtol(),SmollMulti(),LargeMulti()]
    #main06('data/large4.txt',N=10,r=7,min_p=1.5,max_p=1.5)
    #main5('data/large4.txt',droneNum=10,droneList=droneList)
    #plotBCFile("data/__BC")
    #plotResultFile('data/result.txt')
    tryNtimes(10,droneList)
    #calcFlightabelTime()
    
