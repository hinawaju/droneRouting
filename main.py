import random
from field.node import Node
from routing.travellingSalesmanProblem import TravellingSalesmanProblem
from routing import calcThreshold
from routing.singleDP import SingleDP
from routing.doubleDroneRouting import DoubleDR
from routing.vrp import VRP
from model.multicopter import Multi
from model.vtol import Vtol
from field.map import Map
#import openpyxl
from matplotlib import pyplot
from routing.vrpState import VrpState
from routing.singleRouting import SingleRouting

# map criate
def main0(path,N):
    Map.criateMapFile(N,path)
    
# 機体が各ペイロード量で何分飛行できるのか
def main01():
    drone1 = Multi()
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
def main06(path,N,r,p):
    Map.criateLargeMapFile(N,r,p,path)
    
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
def main5(mapFilePath,droneNum):
    map = Map(mapFilePath)
    customerList = map.customerList
    #初期解作成
    initial_state = VrpState(droneNum,map.CN)
    i = 0
    for c in customerList:
        initial_state.miniCustomerMap[i].append(c)
        i += 1
        if i == droneNum:
            i = 0
    
    for j in range(droneNum):
        initial_state.calcCost(j)

    # 初期解表示
    
    for i in range(droneNum):
        print("[",end=" ")
        for n in initial_state.eachFlights[i]:
            print(n.nodeNum,end=", ")
        print("]",initial_state.cost_list[i][0].type,"FT",format(initial_state.cost_list[i][1],'.2f'),"BC",format(initial_state.cost_list[i][2],'.2f'),"payload",format(initial_state.cost_list[i][3],'.2f'))
    
    
    vrp = VRP(initial_state)

    #auto_schedule = vrp.auto(minutes=20) #  時間を20分くらいにしてもらう
    #vrp.set_schedule(auto_schedule)

    state = vrp.anneal()
    print()
    f = open('data/result.txt','a')
    #f.write("drone type, distance, payload, BC"+"\n")
    
    #for i in range(droneNum):
    #    if len(state[0].eachFlights[i])==0:
    #        continue
    #    print("[",end=" ")
    #    for n in state[0].eachFlights[i]:
    #    print(n.nodeNum,end=", ")
    #    print("]",state[0].cost_list[i][0].type,"FT",format(state[0].cost_list[i][1],'.2f'),"BC",format(state[0].cost_list[i][2],'.2f'),"payload",format(state[0].cost_list[i][3],'.2f'))
    
    #state[0].plotRouteFig()
    
    #分析用
    for i in range(droneNum):
        if len(state[0].eachFlights[i])==0:
            continue
        d = 0
        for j in range(len(state[0].eachFlights[i])-1):
            d += map.distance2(state[0].eachFlights[i][j],state[0].eachFlights[i][j+1])
        print(state[0].cost_list[i][0].type,"customer amount",len(state[0].eachFlights[i])-2,"payload",format(state[0].cost_list[i][3],'.2f'),"distance",format(d,'.2f'),"BC",format(state[0].cost_list[i][2],'.2f'))
        f.write(state[0].cost_list[i][0].type+","+str(format(d,'.2f'))+","+str(format(state[0].cost_list[i][3],'.2f'))+","+str(format(state[0].cost_list[i][2],'.2f'))+"\n")
    f.close

def readResultFile(path):
    fig = pyplot.figure()
    ax = fig.add_subplot(111)
    
    f = open(path,'r')
    next(f) #  ファイルの2行目から読み込み
    
    while True: #  マップのファイルから顧客リストを作成
        flightStr = f.readline() #  ファイルから1行読む
        if flightStr == '': #  EOFになったら終了
            break
    
        flightResultList = flightStr.split(',')
        #print(flightResultList[0])
        payload = float(flightResultList[1])
        distance = float(flightResultList[2])
    
        if flightResultList[0] == "multi copter":
            ax.plot(*[payload,distance], 'o', color="red")
        elif flightResultList[0] == "vtol":
            ax.plot(*[payload,distance], 'o', color="blue")
        
    ax.set_xlim([0, 35])
    ax.set_ylim([0, 1.2])
    ax.set_xlabel("flight distance(km)")
    ax.set_ylabel("payload(kg)")
    ax.grid(axis="both")
    
    pyplot.show()
        
        
if __name__ == "__main__":
    main06('data/large5.txt',10,10,0.1)
    main5('data/large5.txt',10)
    readResultFile('data/result.txt')
    #main01()
