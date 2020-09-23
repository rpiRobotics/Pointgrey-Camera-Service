# Overhead Camera Service


import cv2
import cv2.aruco as aruco
import PyCapture2
import numpy as np
import time
import RobotRaconteur as RR
import thread
import threading
import numpy

class overheadCam_imp:
    def __init__(self,cam):
        self._lock = threading.RLock()
        self._cam = cam
        self._fc = [905.3894,906.9932] # Default values for overhead pointgrey cam
        self._cc = [624.0877,481.5628]
        self._kc = [-0.3780,0.1416,0.0027,0.0016,0.0]
        self._camMatrix = np.zeros((3, 3),dtype=np.float64)
        self._camMatrix[0][0] = self._fc[0]
        self._camMatrix[1][1] = self._fc[1]
        self._camMatrix[2][2] = 1.0
        self._camMatrix[0][2] = self._cc[0]
        self._camMatrix[1][2] = self._cc[1]
        self._distCoeff = np.zeros((1, 5), dtype=np.float64)
        self._distCoeff[0][0] = self._kc[0]
        self._distCoeff[0][1] = self._kc[1]
        self._distCoeff[0][2] = self._kc[2]
        self._distCoeff[0][3] = self._kc[3]
        self._distCoeff[0][4] = self._kc[4]
        self._markerSize = 0.127 # default to 5 inches - units in meters
        self._aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)
        self._arucoParams = aruco.DetectorParameters_create()
    def setMarkerSize(self,markerSize):
        with self._lock:
            self._markerSize = markerSize
    def getMarkerSize(self):
        with self._lock:
            markerSize = self._markerSize
            return markerSize
    def setCamParams(self,fc,cc,kc):
        with self._lock:
            self._fc = fc
            self._cc = cc
            self._kc = kc
            
            self._camMatrix[0][0] = self._fc[0]
            self._camMatrix[1][1] = self._fc[1]
            self._camMatrix[2][2] = 1
            self._camMatrix[0][2] = self._cc[0]
            self._camMatrix[1][2] = self._cc[1]

            self._distCoeff[0][0] = self._kc[0]
            self._distCoeff[0][1] = self._kc[1]
            self._distCoeff[0][2] = self._kc[2]
            self._distCoeff[0][3] = self._kc[3]
            self._distCoeff[0][4] = self._kc[4]

    def getCamParams(self):
        with self._lock:
            RRcamParams = RR.RobotRaconteurNode.s.NewStructure("edu.rpi.camService.camParams")
            RRcamParams.fc = self._fc
            RRcamParams.cc = self._cc
            RRcamParams.kc = self._kc
            return RRcamParams
        
    def getObjectPose(self,corners, tag_size, camMatrix, distCoeff):
        with self._lock:
            # AR Tag Dimensions
            objPoints = np.zeros((4, 3), dtype=np.float64)
            objPoints[0,0] = -1*tag_size/2.0 # -1
            objPoints[0,1] = 1*tag_size/2.0 # +1
            objPoints[0,2] = 0.0
            objPoints[1,0] = tag_size/2.0 # +1
            objPoints[1,1] = tag_size/2.0 # +1
            objPoints[1,2] = 0.0
            objPoints[2,0] = tag_size/2.0 # +1
            objPoints[2,1] = -1*tag_size/2.0 # -1
            objPoints[2,2] = 0.0
            objPoints[3,0] = -1*tag_size/2.0 # -1
            objPoints[3,1] = -1*tag_size/2.0 # -1
            objPoints[3,2] = 0.0

            # Get each corner of the tags
            imgPoints = np.zeros((4, 2), dtype=np.float64)
            for i in range(4):
                imgPoints[i, :] = corners[0, i, :]

            # SolvePnP
            retVal, rvec, tvec = cv2.solvePnP(objPoints, imgPoints, camMatrix, distCoeff)
            Rca, b = cv2.Rodrigues(rvec)
            Pca = tvec

            return [Pca, Rca]
    def getImage(self):
        with self._lock:
            self._cam.startCapture()
            try:
                image = self._cam.retrieveBuffer()
            except PyCapture2.Fc2error as fc2Err:
                print "Error retrieving buffer : ", fc2Err

            image = image.convert(PyCapture2.PIXEL_FORMAT.BGR)
            frame = np.array(image.getData(), dtype="uint8").reshape( (image.getRows(), image.getCols(),3) )
            self._cam.stopCapture()

            RRimg = RR.RobotRaconteurNode.s.NewStructure("edu.rpi.camService.Image")
            RRimg.height, RRimg.width, RRimg.channels = frame.shape
            RRimg.data = numpy.frombuffer(frame.tostring(),dtype="u1")
            return RRimg
    def detectMarkers(self):
        with self._lock:
            self._cam.startCapture()
            try:
                image = self._cam.retrieveBuffer()
            except PyCapture2.Fc2error as fc2Err:
                print "Error retrieving buffer : ", fc2Err
            image = image.convert(PyCapture2.PIXEL_FORMAT.BGR)
            frame = np.array(image.getData(), dtype="uint8").reshape( (image.getRows(), image.getCols(),3) )
            self._cam.stopCapture()

            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY) 
            corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, self._aruco_dict, parameters=self._arucoParams)

            IDS = []
            Tmat_temp = []
            if ids is None:
                IDS.append(float('nan'))
                Tmat_temp = [0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,1.0]
            else:
                for m in ids:
                    IDS.append(m[0])
                for m in corners:
                    pc_temp, Rc_temp = self.getObjectPose(m, self._markerSize, self._camMatrix, self._distCoeff) #88.5
                    Tmat_temp.extend([Rc_temp[0][0],Rc_temp[1][0],Rc_temp[2][0],0.0,
                                     Rc_temp[0][1],Rc_temp[1][1],Rc_temp[2][1],0.0,
                                     Rc_temp[0][2],Rc_temp[1][2],Rc_temp[2][2],0.0,
                                     pc_temp[0],pc_temp[1],pc_temp[2],1.0])
                    # Print for troubleshooting:
                    #print(Rc_temp)
                    #print('\n')
                    #print(pc_temp)
                    #print('\n')
                    
                    
        

            detectedMarkers = RR.RobotRaconteurNode.s.NewStructure("edu.rpi.camService.detectedMarkers")
            detectedMarkers.ids = IDS
            detectedMarkers.Tmat = Tmat_temp
            return detectedMarkers



def main():

    bus = PyCapture2.BusManager()
    numCams = bus.getNumOfCameras()
    print "Number of cameras detected: ", numCams
    if not numCams:
                print "Insufficient number of cameras. Exiting..."
                exit()
            
    # Select camera on 0th index
    cam = PyCapture2.Camera()
    uid = bus.getCameraFromIndex(0)
    cam.connect(uid)


    
    RR.RobotRaconteurNode.s.NodeName = "OverheadCam"
    overheadCam = overheadCam_imp(cam)

    t = RR.TcpTransport()
    t.StartServer(5885)
    RR.RobotRaconteurNode.s.RegisterTransport(t)

    try:
        with open('overheadCamService.robdef','r') as f:
            service_def = f.read()
    except:
        print("error1")
    try:
        RR.RobotRaconteurNode.s.RegisterServiceType(service_def)
    except:
        print("error2")
    try:
        RR.RobotRaconteurNode.s.RegisterService("ohcam","edu.rpi.camService.ohcam",overheadCam)
        print("Connect at tcp://localhost:5885/OverheadCam/ohcam")
        raw_input("press enter to quit...\r\n")

    except:
        print("error")

    RR.RobotRaconteurNode.s.Shutdown()
    cam.disconnect()
    cv2.destroyAllWindows()
    

if __name__=='__main__':
    main()
