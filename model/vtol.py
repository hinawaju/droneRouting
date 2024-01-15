from . import airframe

class Vtol(airframe.Airframe):

    def __init__(self):
        super().__init__()
        self.battery_p = 100  # 機体として正しい値は1118880(maxの7割)
        self.battery_J = 1118880
        self.takeOffTime_m = 0.5  # 離着陸にかかる時間
        self.speed_km_m = 1.167  # 機体速度70km/h
        self.maxPayload_kg = 1
        self.type = "vtol"
        self.color = "blue"
        self.alpha_f = 0.22
        self.beta_f = 2.65
        self.alpha_h = 1.03
        self.beta_h = 5.93
    
    #固定翼モードでの1分あたりの消費電力割合(％/分)
    def consum_f(self,payload_kg):
        return self.alpha_f*payload_kg+self.beta_f

    #回転翼モードで行う離着陸1回あたり（15×２s)にかかる消費電力割合（％/１回）
    def consum_h(self,payload_kg):#30sで計算してある式なので×self.takeOffTime_mなどはいらない
        return self.alpha_h*payload_kg+self.beta_h

    def calcBattery_f(self,distance_km,payload_kg):
        battery = self.consum_f(payload_kg)*distance_km/self.speed_km_m + self.consum_h(payload_kg)

        return battery

    def addPayloadBC(self,distance_km,addPayload_kg):#TODO consum_hをちゃんと1分で計算して*takeofftime_sをかける
        battery = self.alpha_f*addPayload_kg*distance_km/self.speed_km_m + self.alpha_h*addPayload_kg

        return battery
    
    def calcFlightTime(self,distance_km):
        return distance_km/self.speed_km_m + self.takeOffTime_m
