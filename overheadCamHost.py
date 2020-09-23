# Overhead Camera Service


import numpy as np
import time
import RobotRaconteur as RR
import thread
import threading
import numpy
from RobotRaconteur.Client import *
RRN.UseNumpy = True

def detectMarkers(cam):
    detectedMarkers = RR.RobotRaconteurNode.s.NewStructure("edu.rpi.camHost.detectedMarkers")
    cameraService = RRN.ConnectService('tcp://localhost:5885/OverheadCam/ohcam')
    while True:
        markers = cameraService.getDetectedMarkers()
        with cam._lock:
            cam._IDS = markers.ids
            cam._Tmat_temp = markers.Tmat

class overheadCamHost_imp:
    def __init__(self):
        self._lock = threading.RLock()
        self._IDS = []
        self._Tmat_temp = []

    def getDetectedMarkers(self):
        detectedMarkers = RR.RobotRaconteurNode.s.NewStructure("edu.rpi.camHost.detectedMarkers")
        detectedMarkers.ids = self._IDS
        detectedMarkers.Tmat = self._Tmat_temp
        with self._lock:
            return detectedMarkers



def main():

    RR.RobotRaconteurNode.s.NodeName = "OverheadCamHost"
    overheadCamHost = overheadCamHost_imp()

    t = RR.TcpTransport()
    t.StartServer(5995)
    RR.RobotRaconteurNode.s.RegisterTransport(t)

    try:
        with open('overheadCamHost.robdef','r') as f:
            service_def = f.read()
    except:
        print("error1")
    try:
        RR.RobotRaconteurNode.s.RegisterServiceType(service_def)
    except:
        print("error2")

    updateMarkers = threading.Thread(target=detectMarkers,args = (overheadCamHost,))
    updateMarkers.setDaemon(True)
    updateMarkers.start()
        
    try:
        RR.RobotRaconteurNode.s.RegisterService("ohcamhost","edu.rpi.camHost.ohcamhost",overheadCamHost)
        print("Connect at tcp://localhost:5995/OverheadCamHost/ohcamhost")
        raw_input("press enter to quit...\r\n")

    except:
        print("error")

    RR.RobotRaconteurNode.s.Shutdown()
    

if __name__=='__main__':
    main()
