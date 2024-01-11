import random
from routing.travellingSalesmanProblem import TravellingSalesmanProblem
from routing.singleRouting import SingleRouting
from model.multicopter import Multi
from model.vtol import Vtol
from model.airframe import Airframe
from field.node import Node
from matplotlib import pyplot

class VrpState():
    
    def __init__(self,droneNum,droneList,allCustomerNum) -> None:
        self.droneNum = droneNum
        self.droneList = droneList
        self.allCustomerNum = allCustomerNum
        self.miniCustomerMap = []
        self.cost_list = [] #各フライトのdrone type, flight time, battery consumption + penalty,payload,battery consumptionをタプルで保持
        self.eachFlights = [] #最終的な各ドローンのルーティング
        for i in range(droneNum):
            self.miniCustomerMap.append([])
            self.cost_list.append((None,None,None,None))
            self.eachFlights.append([])
        self.change_flight_1 = None
        self.change_flight_2 = None
        self.CAP_PENALTY = 30 # batteryとpayload制限を超えた場合にコストに追加するペナルティ
        self.PAYLOAD_PENALTY = 20 # payload制限超えたときにpayload%10*payload_penaltyを追加
        self.BATTERY_PENALTY = 5 #batteryが100を超えた分にペナルティとして追加する倍率 battery_penalty*(BC-100)
        
        
    def move(self):
        #a1 = 移動元
        a1 = random.randint(0,self.droneNum-1) # randint(a,b)はa,b含む範囲内の整数をランダムで返す。ミニマップリストのインデックスと対応させるため0~droneNum-1
        #b1 = 移動先
        b1 = random.randint(0,self.droneNum-1)
        
        count = 0
        possible = True
        while a1 == b1 or len(self.miniCustomerMap[a1]) == 0:#移動元には顧客が入っていないといけない
            if count >= 3:# 選びなおしは2回まで
                possible = False
                break
            a1 = random.randint(0,self.droneNum-1) # randint(a,b)はa,b含む範囲内の整数をランダムで返す。ミニマップリストのインデックスと対応させるため0~droneNum-1
            b1 = random.randint(0,self.droneNum-1)
            count += 1
        
        if possible == False:
            return False
        a2 = random.randint(0,len(self.miniCustomerMap[a1])-1)
            
        self.miniCustomerMap[b1].append(self.miniCustomerMap[a1][a2])
        del self.miniCustomerMap[a1][a2]
            
        self.change_flight_1 = a1
        self.change_flight_2 = b1
        
        return True
    
    def exchange(self):
        a1 = random.randint(0,self.droneNum-1) # randint(a,b)はa,b含む範囲内の整数をランダムで返す。ミニマップリストのインデックスと対応させるため0~droneNum-1
        b1 = random.randint(0,self.droneNum-1)
        
        count = 0

        while a1 == b1 or len(self.miniCustomerMap[a1]) < 1 or len(self.miniCustomerMap[b1]) < 1:#ここでループしてる
            if count >= 3:# 選びなおしは2回まで
                return False
            a1 = random.randint(0,self.droneNum-1) # randint(a,b)はa,b含む範囲内の整数をランダムで返す。ミニマップリストのインデックスと対応させるため0~droneNum-1
            b1 = random.randint(0,self.droneNum-1)
            count += 1
        a2 = random.randint(0,len(self.miniCustomerMap[a1])-1)
        b2 = random.randint(0,len(self.miniCustomerMap[b1])-1)

        self.miniCustomerMap[a1][a2] ,self.miniCustomerMap[b1][b2] = self.miniCustomerMap[b1][b2],self.miniCustomerMap[a1][a2]
        
        self.change_flight_1 = a1
        self.change_flight_2 = b1
        
        return True
    
    def change(self):
        R = random.randint(0,1)

        done = False
        if R == 0:#移動
            done = self.move()
        else:
            done = self.exchange()
        
        if done == True:
            self.calcCost(self.change_flight_1)
            self.calcCost(self.change_flight_2)

        else:
            return
        
    def calcScore(self):#TODO バッテリ消費量の単位を％からJなどに変更
        sum_BC = 0
        for touple in self.cost_list:
            sum_BC += touple[2]
        
        return sum_BC
    
    def calcCost(self,map_id):#TODO バッテリ消費量の単位を％からJなどに変更
        #TODO droneList にはMulti()とかを格納
        
        if len(self.miniCustomerMap[map_id]) == 0:  # self.miniCustomerMap[map_id]が空ベクトルのときTBが作られずserachBestRouting()でエラー吐く
            self.cost_list[map_id] = (Airframe(),0,0,0,0)
            self.eachFlights[map_id] = []
                                           
            #print("顧客数 0")
                                           
            return
        
        eachDroneBCList = []
        penaltyBCList = []
        for drone in self.droneList:
            routing = SingleRouting(self.miniCustomerMap[map_id],drone,self.allCustomerNum)
            routing.criateTBobjectB()
            routing.searchBestRouteObjectB()
            eachDroneBCList.append((routing.BC, routing.FT, routing.bestRoute))
        
        sumPayload = 0
        for c in self.miniCustomerMap[map_id]:
            sumPayload += c.demand
        
        #if len(self.miniCustomerMap[map_id]) >= 12:# １顧客につき最低0.1kgの荷物を運ぶので、制限内で配達できる顧客数は最大10カ所である。焼きなまし法なのである程度制限を超える可能性を残して11顧客まで許容する
        #    self.cost_list[map_id] = (Airframe(),float("inf"),300+2*self.PAYLOAD_PENALTY*len(self.miniCustomerMap[map_id]),sumPayload)
        #    self.eachFlights[map_id] = []
        #    return
        
        #print("multi : "+str(multiBC*Multi().battery_J/100)+", vtol : "+str(vtolBC*Vtol().battery_J/100))
        
        for i in range(len(self.droneList)):
            penaltyBCList.append(eachDroneBCList[i][0])
            if sumPayload > self.droneList[i].maxPayload_kg or eachDroneBCList[i][0] > 100:
                penaltyBCList[i] += self.CAP_PENALTY
                if eachDroneBCList[i][0] > 100:
                    penaltyBCList[i] += (eachDroneBCList[i][0]-100)*self.BATTERY_PENALTY
                if sumPayload > self.droneList[i].maxPayload_kg:
                    penaltyBCList[i] += (sumPayload-self.droneList[i].maxPayload_kg)*10*self.PAYLOAD_PENALTY
            
        minIndex = penaltyBCList.index(min(penaltyBCList))
        self.cost_list[map_id] = (self.droneList[minIndex],eachDroneBCList[i][1],penaltyBCList[minIndex],sumPayload,eachDroneBCList[i][0])
        self.eachFlights[map_id] = eachDroneBCList[i][2]

                                                                                        
        #print("顧客数",len(self.miniCustomerMap[map_id]),"payload",sumPayload)
                                                                                
    
    def plotRouteFig(self):
        fig = pyplot.figure()
        ax = fig.add_subplot(111)

        ax.plot(*[0,0], 'o', color="black") #  デポのプロット
        for map in self.miniCustomerMap:
            for n in map:
                ax.plot(*[n.x,n.y], 'o', color="red")
                ax.text(n.x, n.y,n.demand)

        """
        for flight in self.eachFlights:
            for i in range(len(flight)-1): # 矢印のプロット
                fromNode = flight[i]
                toNode = flight[i+1]
                ax.annotate('', xy=[toNode.x,toNode.y], xytext=[fromNode.x,fromNode.y],
                            arrowprops=dict(shrink=0, width=1, headwidth=8, 
                                            headlength=10, connectionstyle='arc3',
                                            facecolor='gray', edgecolor='gray')
                            )
        """
        
        for i in range(len(self.eachFlights)):
            # 機体によってベクトルを色分け
            color = self.cost_list[i][0].color

            for j in range(len(self.eachFlights[i])-1):
                fromNode = self.eachFlights[i][j]
                toNode = self.eachFlights[i][j+1]
                ax.annotate('', xy=[toNode.x,toNode.y], xytext=[fromNode.x,fromNode.y],
                            arrowprops=dict(shrink=0, width=1, headwidth=8, 
                                            headlength=10, connectionstyle='arc3',
                                            facecolor=color, edgecolor=color)
                            )
        pyplot.show()