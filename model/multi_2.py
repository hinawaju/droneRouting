from . import airframe

class Multi2(airframe.Airframe):
    #smoll drone
    def __init__(self):
        super().__init__()
        self.battery_p = 100  # 機体として正しい値は321120*9/1.4 321120が機体そのままのバッテリー量の7割、今回機体を比較するにあたって機体重量を基準に合わせて9/1.4倍してるのでその他のパラメータも9/1.4倍
        self.battery_J = 540000
        self.takeOffTime_m = 0.5  # 離着陸にかかる時間
        self.speed_km_m = 0.6  # 機体速度10m/s
        self.maxPayload_kg = 0.5
        self.type = "S multi"
        self.color = "yellow"

    # 前進飛行での1分あたりの消費電力割合（％/分)
    def consum_f(self,payload_kg):
        return 0.88*payload_kg + 0.3
        
    # 離着陸１回あたりでのバッテリー消費割合（％）
    def consum_h(self,payload_kg):
        return self.takeOffTime_m * self.consum_f(payload_kg)

    def calcBattery_f(self, distance_km, payload_kg):
        return self.consum_f(payload_kg)*distance_km/self.speed_km_m + self.consum_h(payload_kg)

    def addPayloadBC(self,distance_km,addPayload_kg):
        return 0.88*addPayload_kg * distance_km/self.speed_km_m + 0.88*addPayload_kg * self.takeOffTime_m

    def calcFlightTime(self,distance_km):
        return distance_km/self.speed_km_m + self.takeOffTime_m