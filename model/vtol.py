from . import airframe

class Vtol(airframe.Airframe):

    def __init__(self):
        super().__init__()
        self.battery_p = 100  # 機体として正しい値は1118880(maxの7割)
        self.takeOffTime_m = 0.5  # 離着陸にかかる時間
        self.speed_km_m = 1.083  # 機体速度65km/h
        self.maxPayload_kg = 1
        self.type = "vtol"
    
    #固定翼モードでの1分あたりの消費電力割合(％/分)
    def consum_f(self,payload_kg):
        return 0.334314163*payload_kg+2.296212974

    #回転翼モードで行う離着陸1回あたり（30×２s)にかかる消費電力割合（％/１回）
    def consum_h(self,payload_kg):#30sで計算してある式なので×self.takeOffTime_mなどはいらない
        return 1.0124*payload_kg+8.1491

    def calcBattery_f(self,distance_km,payload_kg):
        battery = self.consum_f(payload_kg)*distance_km/self.speed_km_m + self.consum_h(payload_kg)

        return battery

    def addPayloadBC(self,distance_km,addPayload_kg):
        battery = 0.309102509*addPayload_kg*distance_km/self.speed_km_m + 1.8721*addPayload_kg

        return battery

    def calcFlightTime(self,distance_km):
        return distance_km/self.speed_km_m + self.takeOffTime_m
