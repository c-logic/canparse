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

        inputca = None

        #single frame
        if (pgn >= 59392 and pgn <= 60928) or pgn == 61184 or (pgn >= 61440 and pgn <= 65279) or (pgn >= 65280 and pgn <= 65535) or (pgn >= 126208 and pgn <= 126464):
                if (ca := getattr(self, f"decsp{pgn}",None)) is None and pgn == 61184 or (pgn >= 65280 and pgn <= 65535):
                    inputca = (self.retstandard, msg.data)
                else:
                    inputca = (ca, msg.data)
        #fast packet
        elif pgn == 126720 or (pgn >= 130816 and pgn <= 131071):
            if (data := self.pushfastpacket(pgn,msg.data)) is not None:
                if (ca := getattr(self, f"decfp{pgn}",None)) is None:
                    inputca = (self.retstandard, data)
                else:
                    inputca = (ca, data)
        elif pgn >= 126976 and pgn <= 130815:
            if (ca:= getattr(self, f"decsp{pgn}",None)) is not None:
                inputca = (ca, msg.data)
            elif (ca:= getattr(self, f"decfp{pgn}",None)) is not None:
                if (data := self.pushfastpacket(pgn, msg.data)) is not None:
                    inputca = (ca, data)

        if inputca is not None:
            try:
                data = inputca[0](inputca[1])
            except:
                data = self.retparseerror(inputca[1])
            return (pgn,data)
        return None
    #strings
    def GfixString(self, data, idx, length):
        d = data[idx:idx+length]
        f = d.find(b'\xff') # possible \x00 or '@'
        return data[idx:idx+length] if f<0 else data[idx:idx+f]

    def GvarString(self, data, idx):
        l = data[idx] #length
        t = data[idx+1] #type
        if t == 1:
            return idx+2+l-2, data[idx+2:idx+2+l-2].decode()

    #unsigned int
    def Gu8(self, msg, idx):
        return None if msg[idx] == 0xff else msg[idx]

    def Gu16(self, msg, idx):
        return None if (d := unpack("H",msg[idx:idx+2])[0]) == 0xFFFF else d

    def Gu24(self, msg, idx):
        return None if (d := (unpack("H",msg[idx+1:idx+3])[0]<<8) | msg[idx+2]) == 0xFF_FFFF else d

    def Gu32(self, msg, idx):
        return None if (d := unpack("I",msg[idx:idx+4])[0]) == 0xFFFF_FFFF else d 

    def Gu64(self, msg, idx):
        return None if (d:= unpack("Q",msg[idx:idx+8])[0]) == 0xFFFF_FFFF_FFFF_FFFF else d 

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
        return None if (d := self.Gu8(msg, idx)) is None else d * calc 

    def Gud16(self, msg, idx, calc=1.):
        return None if (d := self.Gu16(msg, idx)) is None else d * calc 

    def Gud24(self, msg,idx, calc=1.):
        return None if (d := self.Gu24(msg, idx)) is None else d * calc 

    def Gud32(self, msg,idx, calc=1.):
        return None if (d := self.Gu32(msg, idx)) is None else d * calc 

    def pushfastpacket(self, pgn, msg):
        sc = msg[0] >> 5 #session counter
        fc = msg[0] & 0x1f #frame counter

        bufferid = pgn*8+sc

        if fc == 0:
            datalen = msg[1]
            if datalen == 0:
                return ""
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
        return None

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

    manufactorcode = {69: "ARKS Enterprises, Inc.", 78: "FW Murphy/Enovation Controls", 80: "Twin Disc", 85: "Kohler Power Systems", 88: "Hemisphere GPS Inc", 116: "BEP Marine", 135: "Airmar", 137: "Maretron", 140: "Lowrance", 144: "Mercury Marine", 147: "Nautibus Electronic GmbH", 148: "Blue Water Data", 154: "Westerbeke", 161: "Offshore Systems (UK) Ltd.", 163: "Evinrude/BRP", 165: "CPAC Systems AB", 168: "Xantrex Technology Inc.", 172: "Yanmar Marine", 174: "Volvo Penta", 175: "Honda Marine", 176: "Carling Technologies Inc. (Moritz Aerospace)", 185: "Beede Instruments", 192: "Floscan Instrument Co. Inc.", 193: "Nobletec", 198: "Mystic Valley Communications", 199: "Actia", 200: "Honda Marine", 201: "Disenos Y Technologia", 211: "Digital Switching Systems", 215: "Xintex/Atena", 224: "EMMI NETWORK S.L.", 225: "Honda Marine", 228: "ZF", 229: "Garmin", 233: "Yacht Monitoring Solutions", 235: "Sailormade Marine Telemetry/Tetra Technology LTD", 243: "Eride", 250: "Honda Marine", 257: "Honda Motor Company LTD", 272: "Groco", 273: "Actisense", 274: "Amphenol LTW Technology", 275: "Navico", 283: "Hamilton Jet", 285: "Sea Recovery", 286: "Coelmo SRL Italy", 295: "BEP Marine", 304: "Empir Bus", 305: "NovAtel", 306: "Sleipner Motor AS", 307: "MBW Technologies", 311: "Fischer Panda", 315: "ICOM", 328: "Qwerty", 329: "Dief", 341: "Boening Automationstechnologie GmbH & Co. KG", 345: "Korean Maritime University", 351: "Thrane and Thrane", 355: "Mastervolt", 356: "Fischer Panda Generators", 358: "Victron Energy", 370: "Rolls Royce Marine", 373: "Electronic Design", 374: "Northern Lights", 378: "Glendinning", 381: "B & G", 384: "Rose Point Navigation Systems", 385: "Johnson Outdoors Marine Electronics Inc Geonav", 394: "Capi 2", 396: "Beyond Measure", 400: "Livorsi Marine", 404: "ComNav", 409: "Chetco", 419: "Fusion Electronics", 421: "Standard Horizon", 422: "True Heading AB", 426: "Egersund Marine Electronics AS", 427: "em-trak Marine Electronics", 431: "Tohatsu Co, JP", 437: "Digital Yacht", 438: "Comar Systems Limited", 440: "Cummins", 443: "VDO (aka Continental-Corporation)", 451: "Parker Hannifin aka Village Marine Tech", 459: "Alltek Marine Electronics Corp", 460: "SAN GIORGIO S.E.I.N", 466: "Veethree Electronics & Marine", 467: "Humminbird Marine Electronics", 470: "SI-TEX Marine Electronics", 471: "Sea Cross Marine AB", 475: "GME aka Standard Communications Pty LTD", 476: "Humminbird Marine Electronics", 478: "Ocean Sat BV", 481: "Chetco Digitial Instruments", 493: "Watcheye", 499: "Lcj Capteurs", 502: "Attwood Marine", 503: "Naviop S.R.L.", 504: "Vesper Marine Ltd", 510: "Marinesoft Co. LTD", 517: "NoLand Engineering", 518: "Transas USA", 529: "National Instruments Korea", 532: "Onwa Marine", 571: "Marinecraft (South Korea)", 573: "McMurdo Group aka Orolia LTD", 578: "Advansea", 579: "KVH", 580: "San Jose Technology", 583: "Yacht Control", 586: "Suzuki Motor Corporation", 591: "US Coast Guard", 595: "Ship Module aka Customware", 600: "Aquatic AV", 605: "Aventics GmbH", 606: "Intellian", 612: "SamwonIT", 614: "Arlt Tecnologies", 637: "Bavaria Yacts", 641: "Diverse Yacht Services", 644: "Wema U.S.A dba KUS", 645: "Garmin", 658: "Shenzhen Jiuzhou Himunication", 688: "Rockford Corp", 704: "JL Audio", 715: "Autonnic", 717: "Yacht Devices", 734: "REAP Systems", 735: "Au Electronics Group", 739: "LxNav", 743: "DaeMyung", 744: "Woosung", 773: "Clarion US", 776: "HMI Systems", 777: "Ocean Signal", 778: "Seekeeper", 781: "Poly Planar", 785: "Fischer Panda DE", 795: "Broyda Industries", 796: "Canadian Automotive", 797: "Tides Marine", 798: "Lumishore", 799: "Still Water Designs and Audio", 802: "BJ Technologies (Beneteau)", 803: "Gill Sensors", 811: "Blue Water Desalination", 815: "FLIR", 824: "Undheim Systems", 838: "TeamSurv", 844: "Fell Marine", 847: "Oceanvolt", 862: "Prospec", 868: "Data Panel Corp", 890: "L3 Technologies", 894: "Rhodan Marine Systems", 896: "Nexfour Solutions", 905: "ASA Electronics", 909: "Marines Co (South Korea)", 911: "Nautic-on", 930: "Ecotronix", 962: "Timbolier Industries", 963: "TJC Micro", 968: "Cox Powertrain", 969: "Blue Seas", 1417: "Revatek", 1850: "Teleflex Marine (SeaStar Solutions)", 1851: "Raymarine", 1852: "Navionics", 1853: "Japan Radio Co", 1854: "Northstar Technologies", 1855: "Furuno", 1856: "Trimble", 1857: "Simrad", 1858: "Litton", 1859: "Kvasar AB", 1860: "MMP", 1861: "Vector Cantech", 1862: "Yamaha Marine", 1863: "Faria Instruments"}
    industrycode = ("Global", "Highway", "Agriculture", "Construction", "Marine", "Industrial")
    def retstandard(self,data):
        return {"mcode": self.Gu16(data,0) & 0x7ff,
        "manufactur": self.manufactorcode[self.Gu16(data,0) & 0x7ff] if (self.Gu16(data,0) & 0x7ff) in self.manufactorcode else None,
        "icode": (self.Gu16(data,0) >> 13) & 7,
        "inducode": self.getname(self.industrycode,(self.Gu16(data,0) >> 13) & 7),
        "data" : " ".join([f"{i:02x}" for i in data])}

    def retparseerror(self,data):
        return {"parseerror": " ".join([f"{i:02x}" for i in data])}
    
    def decsp126992(self, data): #System Time
        return {"sid": self.Gu8(data,0), "source": data[1] & 7, "days": self.Gu16(data,2), "seconds": self.Gud32(data,4,0.0001)}

    watertype = {0:"Fuel", 1:"Water", 2:"GrayWater" ,3:"LiveWell", 4:"Oil", 5:"BlackWater", 6:"FuelGasoline", 14:"Error"}
    def decsp127505(self, data): #Fluid Level
        return {"instance": data[0] >> 4,
                "type": data[0] & 0xf, "typeName": self.getname(self.watertype, data[0] & 0xf, 4),
                "level": self.Gd16(data,1,0.004) ,"capacity": self.Gu32(data,3)}

    dctype = ("Battery", "Alternator", "Converter", "SolarCell", "WindGenerator")
    def decfp127506(self, data): #DC Detailed Status
        return {"sid":data[0], "instance": data[1],
                "type":data[2], "typeName": self.getname(self.dctype, data[2], 8),
                "charge": data[3], "health": data[4] ,"time": self.Gu16(data,5),
                "rippleVoltage": self.Gu16(data,7)*0.01 if self.Gu16(data,7) is not None else None, "capacity": self.Gu16(data,9)}

    chargestate = ("Not_Charging", "Bulk", "Absorption", "Overcharge", "Equalise", "Float", "No_Float", "Constant_VI", "Disabled", "Fault")
    chargemode = ("Standalone", "Primary", "Secondary", "Echo")
    onoff = ("Off", "On", "Error")
    def decsp127507(self, data): #Charger Status
        return {"instance": data[0], "batteryInstance": data[1],
                "operationState": data[2] & 14, "operationStateName": self.getname(self.chargestate, data[2] & 15, 4),
                "chargeMode": data[2] >> 4, "chargeModeName": self.getname(self.chargemode, data[2] >> 4, 4),
                "enabled": data[3] & 3, "enabledname": self.getname(self.onoff, data[3] & 3, 2),
                "equalizationPending": (data[3] >> 2) & 3, "equalizationPendingName": self.getname(self.onoff, (data[3] >> 2) & 3, 2),
                "equatimeRemain": self.Gu16(data,4,60)}

    def decsp127508(self, data): #Battery Status
        return {"sid":data[7], "instance": data[0], "voltage": self.Gd16(data,1,0.01), "current": self.Gd16(data,3,0.1), "temperature": self.Gd16(data,5,0.01)}

    def decsp127744(self, data): #AC Power / Current - Phase A
        return {"sid":data[0], "connection": data[1], "current": self.Gd16(data,2,0.1), "power": self.Gd32(data,4)}

    def decsp127745(self, data): #AC Power / Current - Phase B
        return {"sid":data[0], "connection": data[1], "current": self.Gd16(data,2,0.1), "power": self.Gd32(data,4)}

    def decsp127746(self, data): #AC Power / Current - Phase C
        return {"sid":data[0], "connection": data[1], "current": self.Gd16(data,2,0.1), "power": self.Gd32(data,4)}

    def decsp127747(self, data): #AC Frequency / Voltage - Phase A
        return {"sid":data[0], "connection": data[1], "voltage": self.Gd16(data,2,0.1), "voltage1to2": self.Gd16(data,4,0.1), "frequency": self.Gd16(data,6,0.01)}
    
    def decsp127748(self, data): #AC Frequency / Voltage - Phase B
        return {"sid":data[0], "connection": data[1], "voltage": self.Gd16(data,2,0.1), "voltage2to3": self.Gd16(data,4,0.1), "frequency": self.Gd16(data,6,0.01)}
    
    def decsp127749(self, data): #AC Frequency / Voltage - Phase C
        return {"sid":data[0], "connection": data[1], "voltage": self.Gd16(data,2,0.1), "voltage3to1": self.Gd16(data,4,0.1), "frequency": self.Gd16(data,6,0.01)}

    operatorstate=("Off", "Low Power Mode", "Fault", "Bulk", "Absorption", "Float", "Storage", "Equalize", "Pass thru", "Inverting", "Assisting")
    warningerrorstate=("Good", "Warning", "Error")
    def decsp127750(self, data): #Converter Status
        return {"sid":data[0], "connection": data[1],
                "operationState": data[2], "operatorStateName": self.getname(self.operatorstate, data[2], 8),
                "temperatureState": data[3] & 3, "temperaturStateName": self.getname(self.warningserrorstate, data[3] & 3),
                "overloadState": data[3]>>2 & 3, "overloadStateName": self.getname(self.warningserrorstate, data[3]>>2 & 3),
                "lowDCvoltageState": data[3]>>4 & 3, "lowDCvoltageStateName": self.getname(self.warningserrorstate, data[3]>>4 & 3),
                "rippleState": data[3]>>6 & 0x3, "rippleStateName":  self.getname(self.warningserrorstate, data[3]>>6 & 3)}

    def decsp127751(self, data): #DC Voltage/Current
        return {"sid":data[0], "connection": data[1], "voltage": self.Gud16(data,2,0.1), "current": self.Gd24(data,4,0.01)}

    humsource = ("Inside", "Outside")
    tempsource = ("Sea", "Outside", "Inside", "Engine Room", "Main Cabin", "Live Well", "Bait Well", "Refrigeration",
                        "Heating System", "Dew Point", "Apparent Wind Chill", "Theoretical Wind Chill", "Heat Index",
                        "Freezer", "Exhaust Gas", "Shaft Seal")
    def decsp130311(self, data): #Environmental Parameters
        return {"sid":data[0],
                "tempSource": data[1], "tempSourceName": self.getname(self.tempsource, data[1]),
                "humiditySource": data[2], "humiditySourceName": self.getname(self.humsource, data[2]),
                "temperature": self.Gd16(data,3,.01), "huminity": self.Gd16(data,5,0.004), "pressure": self.Gd16(data,7,100.)}

    def decsp130313(self, data): #Humidity
        return {"sid":data[0], "instance": data[1],
                "source": data[2], "sourcename": self.getname(self.humsource, data[2]),
                "actual": self.Gd16(data,3,.004), "set": self.Gd16(data,5,0.004) }

    pressuresource = ("Atmospheric", "Water", "Steam", "Compressed Air", "Hydraulic", "Filter", "AltimeterSetting", "Oil", "Fuel")
    def decsp130314(self, data): #Actual Pressure
        return {"sid":data[0], "instance": data[1],
                "source": data[2], "sourceName": self.getname(self.pressuresource, data[2]),
                "pressure": self.Gd32(data,3,.004)}

    def decsp130316(self, data): #Temperature Extended Range
        return {"sid":data[0], "instance": data[1],
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

    controllerstate = ("Error Active", "Error Passive", "Bus Off")
    equipstate = ("Operational", "Fault")
    def decsp126993(self,data): #Heartbeat
        return {"offset": self.Gud16(data, 0, 0.001), "sequence": data[1],
                "controllerState1": data[3] & 3, "controllerState1Name": self.getname(self.controllerstate, data[3] & 3),
                "controllerState2": data[3]>>2 & 3, "controllerState2Name": self.getname(self.controllerstate, data[3]>>2 & 3),
                "eqipmentStatus": self.getname(self.equipstate, data[3]>>4 & 3)}

    def decfp126996(self,data): #Product Information
        return {"version": self.Gud16(data, 0, 0.001),
                "productCode": self.Gu16(data, 2),
                "modelId": self.GfixString(data, 4, 32),
                "softwareVersionCode": self.GfixString(data, 4+32, 32),
                "modelVersion": self.GfixString(data, 4+64, 32),
                "modelSerialCode": self.GfixString(data, 4+96, 32),
                "certLevel": data[4+128],
                "loadEquivalency": data[4+128+1]}

    def decfp126998(self,data): #Configuration Information
        nx,st1=self.GvarString(data,0)
        nx,st2=self.GvarString(data,nx)
        nx,st3=self.GvarString(data,nx)
        return {"instDecription1": st1,
                "instDecription2": st2,
                "manufacturerInfo": st3}

    def decfp126464(self,data): #PGN List (Transmit and Receive)
        return {}
    
    def decfp126208(self,data): #NMEA - Request group function
        match data[0]:
            case 0:
                return {"function":"Request"}
            case 1:
                return {"function":"Command"}
            case 2:
                return {"function":"Acknowledge"}
            case 3:
                return {"function":"Read Fields"}
            case 4:
                return {"function":"Read Fields Reply"}
            case 5:
                return {"function":"Write Fields"}
            case 6:
                return {"function":"Write Fields Reply"}
            case _:
                return {"function": data[0]}

    #0xE800-0xEE00 ISO 11783 (protocol)	Single frame
    #59392 - 60928
    def decsp59392(self,data): #ISO Acknowledgement
        return { "control": data[0],
                 "groupfunc": data[1],
                 "pgn#": Gu24(data,5)
                }

    def decsp59904(self,data): #ISO Request
        return { "pgn#": Gu24(data,0) }

    def decsp60160(self,data): #ISO Transport Protocol, Data Transfer
        return { "sid": data[0],
                 "data": " ".join([f"{i:02x}" for i in data[1:]])}

    def decsp60416(self,data): #ISO Transport Protocol, Connection Management
        match data[0]:
            case 0:
                return {"function":"ACK"}
            case 16:
                return {"function":"RTS"}
            case 17:
                return {"function":"CTS"}
            case 19:
                return {"function":"EOM"}
            case 32:
                return {"function":"BAM"}
            case 255:
                return {"function":"Abort"}
            case _:
                return {"function": data[0]}

    def decsp60928(self,data): #ISO Address Claim
        return {"uid": self.Gu32(data,0) & 0x1FFFFF,
                "mcode": (self.Gu32(data,0) >> 21) & 2047,
                "ecuinst": self.Gu8(data,4) & 7,
                "funcinst": (self.Gu8(data,4)>>3) & 31,
                "isofunc": data[5],
                "devclass": data[6]>>1,
                "devclassinst": data[7]>>15,
                "indgroup": (data[7] >> 4) & 7,
                "ArbAddrCap": (data[7] >> 7) & 1,
                }

    magsource = ("Manual", "Automatic Chart", "Automatic Table", "Automatic Calculation", "WMM 2000", "WMM 2005", "WMM 2010", "WMM 2015", "WMM 2020")

    #0x1F000-0x1FEFF Standardized Mixed single/fast
    #126976 - 130815
    def decsp127258(self,data): #Magnetic Variation
        return {"sid":data[0],
                "source": self.getname(self.magsource,data[1] & 15),
                "ageofservice": self.Gu16(data,2),
                "variantion": self.Gd16(data,4,0.0001)
                }

    yesno = ("NO", "YES")
    batttype = ( "Flooded", "Gel", "AGM")
    battvolt = ( 6, 12, 24, 32, 36, 42, 48 )
    battchem = ("Pb", "Li", "NiCd", "ZnO", "NiMH")
    def decfp127513(self,data): #Battery Configuration Status
        return {"inst": data[0],
                "type": self.getname(self.batttype,data[1] & 15),
                "equalsupport": self.getname(self.yesno,(data[1]>>4) & 3),
                "nomvol": self.getname(self.battvolt, data[2] & 15),
                "chem": self.getname(self.battchem, (data[2]>>4) & 15),
                "cap": self.Gu16(data,3),
                "tempcoef": self.Gi8(data,5),
                "peukert": self.Gud8(data,6),
                "chargeeff": self.Gi8(data,7)
                }

    def decsp128267(self,data): #Water Depth
        return {"sid":data[0],
                "depth": self.Gud32(data,1,0.01),
                "offset": self.Gd16(data,5,0.001),
                "range": self.Gud8(data,7,10)}

    def decsp129025(self,data): #Position, Rapid Update
        return {"lat": self.Gd32(data,0,0.0000001),
                "lon": self.Gd32(data,4,0.0000001)}

    dirref = ("True", "Magnetic")
    def decsp129026(self,data): #COG & SOG, Rapid Update
        return {"sid":data[0],
                "COGref": self.getname(self.dirref,data[1] & 3),
                "COG": self.Gud16(data,2,0.0001),
                "SOG": self.Gud16(data,2,0.01)}

    def decfp129029(self,data): #GNSS Position Data
        return {}

    def decsp129283(self,data): #Cross Track Error
        return {}

    def decfp129284(self,data): #Navigation Data
        return {}

    def decsp129539(self,data): #GNSS DOPs
        return {}

    def decfp129285(self,data): #Navigation - Route/WP Information
        return {}

    rangeres = ("Range residuals were used to calculate data", "Range residuals were calculated after the position")
    satstat = ("Not tracked", "Tracked", "Used", "Not tracked+Diff", "Tracked+Diff", "Used+Diff")
    def decfp129540(self,data): #GNSS Sats in View
        sat = []
        if data[2]<=253:
            for i in range(data[2]):
                sat.append({"PRN": data[3+12*i],
                            "elevation": self.Gd16(data,4+12*i,0.0001),
                            "azimuth": self.Gd16(data,6+12*i,0.0001),
                            "snr": self.Gud16(data,8+12*i,0.01),
                            "rangeresid": self.Gd32(data,10+12*i),
                            "status": self.getname(self.satstat,data[11+12*i] & 15),
                            })
        return {"sid": data[0],
                "range": self.getname(self.rangeres, data[1] & 3),
                "satinview": data[2],
                "satlist": sat
                }

    windref = ("True (ground referenced to North)", "Magnetic (ground referenced to Magnetic North)", "Apparent", "True (boat referenced)", "True (water referenced)")
    def decsp130306(self,data): #wind data
        return {"sid": data[0],
                "speed": self.Gud16(data,1,0.01), #m/s
                "angle": self.Gud16(data,3,0.0001), #rad
                "ref": self.getname(self.windref,data[5] & 7)}

    #0x1FF00-0x1FFFF: Manufacturer Specific fast-packet non-addressed
    #130816 - 131071
##    def decfp130822(self,data): #Navico
##        return self.retstandard(data)
##
##    def decfp130824(self,data): #B&G: key-value data
##        return self.retstandard(data)
##
##    def decfp130840(self,data): #Simnet: Data User Group Configuration
##        return self.retstandard(data)
##
##    def decfp130846(self,data): #Simnet: Parameter Set
##        return self.retstandard(data)
##
##    def decfp130850(self,data): #Simnet: AP Command
##        return self.retstandard(data)

    #0xEF00 Manufacturer Proprietary single-frame addressed
    #61184
    def decsp61184(self,data): #Simnet: AP Command
        return self.retstandard(data)

    #0xFF00-0xFFFF: Manufacturer Proprietary single-frame non-addressed
    #65280 - 65535
##    def decsp65280(self,data): #Furuno: Heave
##        return self.retstandard(data)
##
##    def decsp65288(self,data): #Seatalk: Alarm
##        return self.retstandard(data)
##
##    def decsp65294(self,data): #Seatalk: Alarm
##        return self.retstandard(data)
##
##    def decsp65313(self,data): #Seatalk: Alarm
##        return self.retstandard(data)
##
##    def decsp65317(self,data): #Seatalk: Alarm
##        return self.retstandard(data)

nmob=nmea2000(can.Bus(interface="ixxat",channel=0,bitrate=250000))

while True:
    i=nmob.receivenext()
    if i is not None:
        print(i)

