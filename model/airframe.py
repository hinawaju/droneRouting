class Airframe:
    
    def __init__(self):
        self.speed_km_m = 0.8333  # 機体速度50km/h
        self.maxPayload_kg = 1
        self.type = "null"

    def calcBattery_f(self,distance,payload_kg):
        pass

    def addPayloadBC():
        pass

    def calcFlightTime(self,distance_km):
        return distance_km/self.speed_km_m + self.takeOffTime_m

