#import sys
#import os
#sys.path.append(os.path.abspath("."))

import random
import math
from . import node #  singleDPを実行するときはこうしないとエラーでる
#import node #  新しいマップファイルを作成するときはこっちじゃないとエラー出る(main0をつくったのでもういらないはず)
from matplotlib import pyplot

class Map:

    def __init__(self,mapFilePass) -> None:
        self.customerList = [] #  配達先のノードリスト
        self.nodeList = [] #  customerList + デポ
        self.dMatrix = [] #  距離の表
        self.CN = 0
        self.N = 0
        #self.maxXY = 0
        self.r = 0
        self.depo = node.Node(0,0,0,0)
        self.nodeList.append(self.depo)
        self.readMapFile(mapFilePass)
        self.criateDmatrix()
    
    #  ランダムマップ作成
    # node = 'x座標, y座標, demand 'のリスト作成
    @staticmethod
    def criateMapFile(N:int,path):
        #f = open('../data/map1.txt','w') #  fieldのディレクトリから実行する場合はこっち
        #f = open('data/map3.txt','w') #  mainの階層から実行するときはこっち
        f = open(path,'w')
        f.write("x-axis, y-axis, demand")
        maplist = []

        for i in range(N) :
            f.write("\n")
            x_rand = random.randint(1,5)
            y_rand = random.randint(1,5)
            
            while True:
                flag = 1
                for x,y in maplist:
                    if x==x_rand and y==y_rand:
                        flag = 0
                        break
                if flag == 1:
                    break
                elif flag == 0:
                    x_rand = random.randint(1,5)
                    y_rand = random.randint(1,5)
            maplist.append((x_rand,y_rand))
            demand = random.randint(1,2)/10
            print("node_num : ", i+1, ", x : ", x_rand, ", y : ", y_rand, ", demand : ", demand,)
            nodeStr = str(x_rand)+","+str(y_rand)+","+str(demand)
            f.write(nodeStr)

        f.close()
    
    def criateLargeMapFile(N:int,r,p,path):
        f = open(path,'w')
        f.write(str(2*r)+", x-axis, y-axis, demand")
        maplist = []
        _r = -1*r

        for i in range(N) :
            f.write("\n")
            x_rand = random.randint(_r,r)
            y_rand = random.randint(_r,r)
            
            while True:
                flag = 1
                for x,y in maplist:
                    if (x==x_rand and y==y_rand ) or (x_rand==0 and y_rand == 0):
                        flag = 0
                        break
                if flag == 1:
                    break
                elif flag == 0:
                    x_rand = random.randint(_r,r)
                    y_rand = random.randint(_r,r)
            maplist.append((x_rand,y_rand))
            demand = random.randint(1,p*10)/10
            #print("node_num : ", i+1, ", x : ", x_rand, ", y : ", y_rand, ", demand : ", demand,)
            nodeStr = str(x_rand)+","+str(y_rand)+","+str(demand)
            f.write(nodeStr)

        f.close()
                                                                             
    def readMapFile(self,fileName):
        f = open(fileName,'r')
        firstStr = f.readline()
        firstList = firstStr.split(',')
        self.r = int(firstList[0])
        nodeNum = 1 #  nodeListのインデックスと対応

        while True: #  マップのファイルから顧客リストを作成
            nodeStr = f.readline() #  ファイルから1行読む
            if nodeStr == '': #  EOFになったら終了
                break
            nodeList = nodeStr.split(',') #  カンマで分割してx座標,y座標,demandを取得
            x = int(nodeList[0])
            #if x > self.maxXY:
            #    self.maxXY = x
            y = int(nodeList[1])
            #if y > self.maxXY:
            #    self.maxXY = y
            demand = float(nodeList[2])
            n = node.Node(nodeNum,x,y,demand) #  nodeクラスに変換
            self.customerList.append(n) #  顧客リストに追加
            self.nodeList.append(n)
            #print("node_num : ", nodeNum, ", x : ", x, ", y : ", y, ", demand : ", demand)
            nodeNum += 1
            self.CN += 1
        self.N = nodeNum
        f.close()
        
    def showMap(self):
        fig = pyplot.figure()
        ax = fig.add_subplot(111)

        ax.plot(*[0,0], 'o', color="blue") #  デポのプロット
        for p in self.customerList: #  ノードのプロット
            ax.plot(*[p.x,p.y], 'o', color="red")
            ax.text(p.x, p.y,p.demand)
            
        #ax.set_xlim([0, 1.2*self.maxXY])
        #ax.set_ylim([0, 1.2*self.maxXY])
        ax.set_xlim([-8, 8])
        ax.set_ylim([-8, 8])

        pyplot.show()

    def calcSumDemand(self):
        sumDemand = 0
        for c in self.customerList:
            sumDemand += c.demand
        
        return sumDemand
        
    def distance(self,fromNodeNum,toNodeNum):
        fromNode = self.nodeList[fromNodeNum]
        toNode = self.nodeList[toNodeNum]
        return math.sqrt((fromNode.x - toNode.x)**2 + (fromNode.y - toNode.y)**2)
    
    def distance2(self,fromNode,toNode):
        return math.sqrt((fromNode.x - toNode.x)**2 + (fromNode.y - toNode.y)**2)

    def criateDmatrix(self):
        for i in range(self.N):
            dList = []
            for j in range(self.N):
                d = self.distance(i,j)
                dList.append(d)
            self.dMatrix.append(dList)
