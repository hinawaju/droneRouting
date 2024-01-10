from simanneal import Annealer
import random
from model.multicopter import Multi
from model.vtol import Vtol

class VRP(Annealer):
    
    def __init__(self, state):
        self.possible_state = None
        self.best_score = float("inf")
        super(VRP,self).__init__(state)
    
    def move(self):# 差分を返すと高速化？できるらしい
        self.state.change()
        #return super().move()
    
    def energy(self):# 返り値を最小化する
        sum_BC = self.state.calcScore()
        if self.best_score > sum_BC:
            self.best_score = sum_BC
        return sum_BC