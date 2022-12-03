#import sys
#import os
#sys.path.append(os.path.abspath("."))

import random
import math
from . import node #  singleDPを実行するときはこうしないとエラーでる
#import node #  新しいマップファイルを作成するときはこっちじゃないとエラー出る

class Map:

    def __init__(self) -> None:
        self.cList = [] #  顧客リスト
        self.N = 0
    
    #  ランダムマップ作成
    # node = 'x座標, y座標, demand 'のリスト作成
    @staticmethod
    def criateMapFile(N:int):
        f = open('../data/map2.txt','w')
        f.write("x-axis, y-axis, demand")

        for i in range(N) :
            f.write("\n")
            x = random.randint(100,1500)
            y = random.randint(100,1500)
            demand = random.randint(1,4)/10
            print("node_num : ", i+1, ", x : ", x, ", y : ", y, ", demand : ", demand,)
            nodeStr = str(x)+","+str(y)+","+str(demand)
            f.write(nodeStr)

        f.close()
    

    def readMapFile(self,fileName):
        f = open(fileName,'r')
        next(f) #  ファイルの2行目から読み込み
        node_num = 1 #  ２進文字列にしたとき、-node_numで場所を参照できるように

        while True: #  マップのファイルから顧客リストを作成
            nodeStr = f.readline() #  ファイルから1行読む
            if nodeStr == '': #  EOFになったら終了
                break
            nodeList = nodeStr.split(',') #  カンマで分割してx座標,y座標,demandを取得
            x = int(nodeList[0])
            y = int(nodeList[1])
            demand = float(nodeList[2])
            n = node.Node(node_num,x,y,demand) #  nodeクラスに変換
            self.cList.append(n) #  顧客リストに追加
            print("node_num : ", node_num, ", x : ", x, ", y : ", y, ", demand : ", demand)
            node_num += 1
        f.close()

    @staticmethod
    def distance(from_node,to_node):
        return math.sqrt((from_node.x - to_node.x)**2 + (from_node.y - to_node.y)**2)

if __name__ == "__main__":
    Map.criateMapFile(11)