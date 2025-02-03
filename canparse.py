import can
from math import ceil
from struct import unpack

class nmea2000:
    buffer = {}
    bus = None

    def __init__(self, canbus):
        self.bus=canbus

    def receivenext(self):
        msg = self.bus.recv()
        priority, source, pgn = self.decodepgn(msg.arbitration_id)

        try:
            ca = getattr(self, f"decsp{pgn}")
        except:
            ca = None

        if ca is not None:
            return ca(msg.data)
        else:
            try:
                ca = getattr(self, f"decfp{pgn}")
            except:
                ca = None

            if ca is not None:
                data = self.pushfastpacket(pgn,msg.data)
                return ca(data) if data is not None else None

    #unsigned int
    def Gu8(self, msg, idx):
        return None if msg[idx] == 0xff else msg[idx]

    def Gu16(self, msg, idx):
        d = unpack("H",msg[idx:idx+2])[0]
        return None if d == 0xFFFF else d

    def Gu24(self, msg, idx):
        d = (unpack("H",msg[idx+1:idx+3])[0]<<8) | msg[idx+2]
        return None if d == 0xFF_FFFF else d

    def Gu32(self, msg, idx):
        d = unpack("I",msg[idx:idx+4])[0]
        return None if d == 0xFFFF_FFFF else d 

    def Gu64(self, msg, idx):
        d = unpack("Q",msg[idx:idx+8])[0]
        return None if d == 0xFFFF_FFFF_FFFF_FFFF else d 

    #signed int
    def Gi8(self, msg, idx):
        return None if msg[idx] == 0x7f else unpack("b",msg[idx:idx+1])[0]

    def Gi16(self, msg, idx):
        return None if msg[idx:idx+2] == bytearray((0xff,0x7f)) else unpack("h",msg[idx:idx+2])[0]

    def Gi24(self, msg, idx):
        return None if msg[idx:idx+3] == bytearray((0xff,0xff,0x7f)) else (unpack("h",msg[idx+1:idx+3])[0]<<8) | msg[idx+2]

    def Gi32(self, msg, idx):
        return None if msg[idx:idx+4] == bytearray((0xff,0xff,0xff,0x7f)) else unpack("i",msg[idx:idx+4])[0]

    def Gi64(self, msg, idx):
        return None if msg[idx:idx+8] == bytearray((0xff,0xff,0xff,0xff,0xff,0xff,0xff,0x7f)) else unpack("q",msg[idx:idx+8])[0]

    #signed double
    def Gd8(self, msg, idx, calc=1.):
        d = self.Gi8(msg, idx)
        return None if d is None else d * calc 

    def Gd16(self, msg, idx, calc=1.):
        d = self.Gi16(msg, idx)
        return None if d is None else d * calc 

    def Gd24(self, msg,idx, calc=1.):
        d = self.Gi24(msg, idx)
        return None if d is None else d * calc 

    def Gd32(self, msg,idx, calc=1.):
        d = self.Gi32(msg, idx)
        return None if d is None else d * calc 

    def Gd64(self, msg,idx, calc=1.):
        d = self.Gi64(msg, idx)
        return None if d is None else d * calc 

    #unsigned double
    def Gud8(self, msg, idx, calc=1.):
        d = self.Gu8(msg, idx)
        return None if d is None else d * calc 

    def Gud16(self, msg, idx, calc=1.):
        d = self.Gu16(msg, idx)
        return None if d is None else d * calc 

    def Gud24(self, msg,idx, calc=1.):
        d = self.Gu24(msg, idx)
        return None if d is None else d * calc 

    def Gud32(self, msg,idx, calc=1.):
        d = self.Gu32(msg, idx)
        return None if d is None else d * calc 

    def pushfastpacket(self, pgn, msg):
        sc = msg[0] >> 5 #session counter
        fc = msg[0] & 0x1f #frame counter

        bufferid = pgn*8+sc
        ret = None

        if fc == 0:
            datalen = msg[1]
            data=[None]*int(ceil((datalen - 6) / 7)+1)
            data[0]=msg[2:]
            self.buffer[bufferid]=(datalen, data)
        else:
            pnt = 7*fc+6
            if bufferid in self.buffer:
                self.buffer[bufferid][1][fc]=msg[1:]
                if all(v is not None for v in self.buffer[bufferid][1]):
                    ret = b''.join(self.buffer[bufferid][1])[:self.buffer[bufferid][0]]
                    del self.buffer[bufferid]
        return ret

    def getname(self, tup, idx, en = None, notfound = "Unavailable"):
        if isinstance(tup, tuple):
            if en is None:
                return tup[idx] if idx < len(tup) else None
            else:
                return notfound if idx == ((1<<en)-1) else tup[idx] if idx < len(tup) else None
        elif isinstance(tup, dict):
            if en is None:
                return tup[idx] if idx in tup else None
            else:
                return notfound if idx == ((1<<en)-1) else tup[idx] if idx in tup else None

    def decodepgn(self, arb):
        return (arb & 0x1c000000) >> 26, arb & 0x000000ff, (arb & 0x3ffff00) >> 8

    def decsp126992(self, data): #System Time
        return {"pgn":126992, "sid": self.Gu8(data,0), "source": data[1] & 7, "days": self.Gu16(data,2), "seconds": self.Gud32(data,4,0.0001)}

    watertype = {0:"Fuel", 1:"Water", 2:"GrayWater" ,3:"LiveWell", 4:"Oil", 5:"BlackWater", 6:"FuelGasoline", 14:"Error"}
    def decsp127505(self, data): #Fluid Level
        return {"pgn":127505, "instance": data[0] >> 4,
                "type": data[0] & 0xf, "typeName": self.getname(self.watertype, data[0] & 0xf, 4),
                "level": self.Gd16(data,1,0.004) ,"capacity": self.Gu32(data,3)}

    dctype = ("Battery", "Alternator", "Converter", "SolarCell", "WindGenerator")
    def decfp127506(self, data): #DC Detailed Status
        return {"pgn":127506, "sid":data[0], "instance": data[1],
                "type":data[2], "typeName": self.getname(self.dctype, data[2], 8),
                "charge": data[3], "health": data[4] ,"time": self.Gu16(data,5),
                "rippleVoltage": self.Gu16(data,7)*0.01, "capacity": self.Gu16(data,9)}

    chargestate = ("Not_Charging", "Bulk", "Absorption", "Overcharge", "Equalise", "Float", "No_Float", "Constant_VI", "Disabled", "Fault")
    chargemode = ("Standalone", "Primary", "Secondary", "Echo")
    onoff = ("Off", "On", "Error")
    def decsp127507(self, data): #Charger Status
        return {"pgn": 127507, "instance": data[0], "batteryInstance": data[1],
                "operationState": data[2] & 14, "operationStateName": self.getname(self.chargestate, data[2] & 15, 4),
                "chargeMode": data[2] >> 4, "chargeModeName": self.getname(self.chargemode, data[2] >> 4, 4),
                "enabled": data[3] & 3, "enabledname": self.getname(self.onoff, data[3] & 3, 2),
                "equalizationPending": (data[3] >> 2) & 3, "equalizationPendingName": self.getname(self.onoff, (data[3] >> 2) & 3, 2),
                "equatimeRemain": self.Gu16(data,4,60)}

    def decsp127508(self, data): #Battery Status
        return {"pgn":127508, "sid":data[7], "instance": data[0], "voltage": self.Gd16(data,1,0.01), "current": self.Gd16(data,3,0.1), "temperature": self.Gd16(data,5,0.01)}

    def decsp127744(self, data): #AC Power / Current - Phase A
        return {"pgn":127744, "sid":data[0], "connection": data[1], "current": self.Gd16(data,2,0.1), "power": self.Gd32(data,4)}

    def decsp127745(self, data): #AC Power / Current - Phase B
        return {"pgn":127745, "sid":data[0], "connection": data[1], "current": self.Gd16(data,2,0.1), "power": self.Gd32(data,4)}

    def decsp127746(self, data): #AC Power / Current - Phase C
        return {"pgn":127746, "sid":data[0], "connection": data[1], "current": self.Gd16(data,2,0.1), "power": self.Gd32(data,4)}

    def decsp127747(self, data): #AC Frequency / Voltage - Phase A
        return {"pgn":127747, "sid":data[0], "connection": data[1], "voltage": self.Gd16(data,2,0.1), "voltage1to2": self.Gd16(data,4,0.1), "frequency": self.Gd16(data,6,0.01)}
    
    def decsp127748(self, data): #AC Frequency / Voltage - Phase B
        return {"pgn":127748, "sid":data[0], "connection": data[1], "voltage": self.Gd16(data,2,0.1), "voltage2to3": self.Gd16(data,4,0.1), "frequency": self.Gd16(data,6,0.01)}
    
    def decsp127749(self, data): #AC Frequency / Voltage - Phase C
        return {"pgn":127749, "sid":data[0], "connection": data[1], "voltage": self.Gd16(data,2,0.1), "voltage3to1": self.Gd16(data,4,0.1), "frequency": self.Gd16(data,6,0.01)}

    operatorstate=("Off", "Low Power Mode", "Fault", "Bulk", "Absorption", "Float", "Storage", "Equalize", "Pass thru", "Inverting", "Assisting")
    warningerrorstate=("Good", "Warning", "Error")
    def decsp127750(self, data): #Converter Status
        return {"pgn":127750, "sid":data[0], "connection": data[1],
                "operationState": data[2], "operatorStateName": self.getname(self.operatorstate, data[2], 8),
                "temperatureState": data[3] & 3, "temperaturStateName": self.getname(self.warningserrorstate, data[3] & 3),
                "overloadState": data[3]>>2 & 3, "overloadStateName": self.getname(self.warningserrorstate, data[3]>>2 & 3),
                "lowDCvoltageState": data[3]>>4 & 3, "lowDCvoltageStateName": self.getname(self.warningserrorstate, data[3]>>4 & 3),
                "rippleState": data[3]>>6 & 0x3, "rippleStateName":  self.getname(self.warningserrorstate, data[3]>>6 & 3)}

    def decsp127751(self, data): #DC Voltage/Current
        return {"pgn":127751, "sid":data[0], "connection": data[1], "voltage": self.Gud16(data,2,0.1), "current": self.Gd24(data,4,0.01)}

    humsource = ("Inside", "Outside")
    tempsource = ("Sea", "Outside", "Inside", "Engine Room", "Main Cabin", "Live Well", "Bait Well", "Refrigeration",
                        "Heating System", "Dew Point", "Apparent Wind Chill", "Theoretical Wind Chill", "Heat Index",
                        "Freezer", "Exhaust Gas", "Shaft Seal")
    def decsp130311(self, data): #Environmental Parameters
        return {"pgn":130311, "sid":data[0],
                "tempSource": data[1], "tempSourceName": self.getname(self.tempsource, data[1]),
                "humiditySource": data[2], "humiditySourceName": self.getname(self.humsource, data[2]),
                "temperature": self.Gd16(data,3,.01), "huminity": self.Gd16(data,5,0.004), "pressure": self.Gd16(data,7,100.)}

    def decsp130313(self, data): #Humidity
        return {"pgn":130313, "sid":data[0], "instance": data[1],
                "source": data[2], "sourcename": self.getname(self.humsource, data[2]),
                "actual": self.Gd16(data,3,.004), "set": self.Gd16(data,5,0.004) }

    presssource = ("Atmospheric", "Water", "Steam", "Compressed Air", "Hydraulic", "Filter", "AltimeterSetting", "Oil", "Fuel")
    def decsp130314(self, data): #Actual Pressure
        return {"pgn":130314, "sid":data[0], "instance": data[1],
                "source": data[2], "sourceName": self.getname(self.presssource, data[2]),
                "pressure": self.Gd32(data,3,.004)}

    def decsp130316(self, data): #Temperature Extended Range
        return {"pgn":130316, "sid":data[0], "instance": data[1],
                "source": data[2], "sourceName": self.getname(self.tempsource, data[2]),
                "actual": self.Gd24(data,3,.001), "set": self.Gd16(data,6,0.1)}

    def decsp127501(self,data): #Binary Switch Bank Status
        d = {"instance": data[0]}
        for t in range(1,29):
            num = (t-1) // 4
            dat = data[num+1]
            for t2 in range(0, 4):
                state = (dat >> (t2*2)) & 3
                d[f"indicatorState{num*4+t2+1}"] = state
                d[f"indicatorState{num*4+t2+1}Name"] = self.getname(self.onoff, state, 2)
        return d

    def decsp127502(self,data): #Binary Switch Bank Control
        d = {"instance": data[0]}
        for t in range(1,29):
            num = (t-1) // 4
            dat = data[num+1]
            for t2 in range(0, 4):
                state = (dat >> (t2*2)) & 3
                d[f"switchState{num*4+t2+1}"] = state
                d[f"switchState{num*4+t2+1}Name"] = self.getname(self.onoff, state, 2)
        return d

nmob=nmea2000(can.Bus(interface="ixxat",channel=0,bitrate=250000))


while True:
    i=nmob.receivenext()
    if i is not None:
        print(i)

