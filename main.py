"""MA_Intersection_Calculation

Code to caluculate all intersections between with multiple cameras detectet
objects.

1) Load Settings
2) Load Camera Calibration
3) consumeKafka Data
3.1) Calc Intersections

Time: Object**n*time
"""
import json, math, cv2, datetime, time
import numpy as np
from kafka import KafkaConsumer, KafkaProducer
import urllib.request, json
from scipy.spatial.transform import Rotation as R
from socket import timeout
from urllib.error import HTTPError, URLError
import sys

### Define Cam Class
class Camera:
    """Define Camera Objects

    Important: MA_Aruco_Calibration needs to be executed first. Cams have to be
    up and running for httpRequest of newest calibration data.

    Opening up a httpRequest on cams Nginx reverse Proxy. Receiving latest
    calibration data, including tvec, rvec, worldvec.
    """
    def __init__(self, id, configPath, CalibrationDataOrigin, calibrationMethod):
        self.id = id
        self.configPath = configPath

        ### Run init methods
        self.loadCameraParams()

        if (CalibrationDataOrigin == 'local'):
            self.loadLocalCalibrationData()
        elif (CalibrationDataOrigin == 'cams'):
            self.loadExternCalibrationData()

        ### Decide between different calibrations
        if (calibrationMethod == 'aruco'):
            self.calculatePositionArucoMarker()
        else:
            print("Choosen calibration method '" + calibrationMethod + "' not exists!")
            sys.exit(0)

    ### Camera Methods
    def loadCameraParams(self):
        with open(self.configPath, 'r') as f:
            config = json.load(f)
            self.intrinsicMatrix = np.array(config['detection']['cams'][self.id]['intrinsicMatrix'], dtype="float")
            self.distCoeffs = np.array(config['detection']['cams'][self.id]['dist'], dtype="float")
            self.focalLength = float(config['detection']['cams'][self.id]['focal_length'])
            self.rotationMarker = np.array(config['detection']['cams'][self.id]['rotation'], dtype="float")
            self.translationMarker = np.array(config['detection']['cams'][self.id]['translation'], dtype="float")

    def loadLocalCalibrationData(self):
        with open(self.configPath, 'r') as f:
            config = json.load(f)
            self.rotVec = np.array(config['detection']['cams'][self.id]['rot_vec'], dtype="float")
            self.transVec = np.array(config['detection']['cams'][self.id]['tvec'], dtype="float")
            self.airDistance = float(config['detection']['cams'][self.id]['distance'])

    def loadExternCalibrationData(self):
        response = urllib.request.urlopen(url, timeout=10).read().decode('utf-8')
        self.rotVec = np.fromstring(string[1], dtype=float, sep=',') # rvec
        self.transVec = np.fromstring(string[3], dtype=float, sep=',') # world rtranslation
        self.airDistance = float(string[2])

    def calculatePositionArucoMarker(self):
        ### Current marker is standing -> transform into lying pattern
        Rx = np.array([[1, 0, 0],[0, 0, 1],[0, -1, 0]])

        ### Apply additional rotation for 90° gap between markers
        self.world2cam = cv2.Rodrigues(self.rotVec.T)[0] @ Rx @ self.rotationMarker
        self.cam2world = np.linalg.inv(self.world2cam) # reprojection

        #### Translation
        self.transVecGlobal = -self.world2cam.T @ self.transVec  + self.translationMarker

### Define Ray Class
class Ray:
    """Calculate rays, based on DeepStramSDK interferencings payload

    Object Position based on DeepStreams Payload Definition. HttpRequset from
    cameras active in the system. Using intrinsic Matrix to remap imagePoints.
    Camera explizit undistortion. Transformation into world coordinate system.
    """
    def __init__(self, pos, camData):
        self.pos = pos # Define Current Position on cam frame
        self.camData = camData
        self.calc()

    def calc(self):
        # DeepstramSDK Payload Form: ID [0], Left [1], Top[2], Right[3], Bottom[4], Category[5]
        x_center = int(self.pos[1]) + (int(self.pos[3]) - int(self.pos[1])) / 2 # Left + (Right - Left) / 2
        y_center = int(self.pos[2]) + (int(self.pos[4]) - int(self.pos[2])) / 2 # Top border + height / 2

        ### Undist for PixelPoint undistortion
        cam_m = self.camData.intrinsicMatrix
        dist_c = self.camData.distCoeffs
        pts = np.array([x_center, y_center])
        dst = cv2.undistortPoints(pts, cam_m, dist_c)

        ### Resolution is normalized to fx, fy out of intrinsic matrix, redefine
        x_center = 953 + 1126 * dst[0][0][0]
        y_center = 535 + 1128 * dst[0][0][1]

        ### Assume calcualtion on intrinsic calibration
        x_middle = self.camData.intrinsicMatrix[0][2]
        y_middle = self.camData.intrinsicMatrix[1][2]

        ### Calculate ray between cam and imagePoint, -> squared focal length
        pbe = np.array([[int(x_center) - x_middle], [int(y_center) - y_middle], [self.camData.focalLength]])
        pbe = (1 / np.linalg.norm(pbe)) * pbe # normalize

        ### Transform into global coordinate system
        self.Pbe = - self.camData.world2cam.T @ pbe + self.camData.transVecGlobal

### Define Intersection Class
class Intersection:
    """Calculate distance between calculated rays

    Object Position based on DeepStreams Payload Definition. HttpRequset from
    cameras active in the system. Using intrinsic Matrix to remap imagePoints.
    Camera explizit undistortion. Transformation into world coordinate system.
    """
    def __init__(self, rayCam1, rayCam2, camsData):
        self.rayCam1 = rayCam1
        self.rayCam2 = rayCam2
        self.camsData = camsData
        self.calc()

    def calc(self):
        ### Get Data for Ray1
        P1 = self.camsData[0].transVecGlobal
        R1 = np.array(np.add(self.rayCam1.Pbe,-P1)) # Object <-> Cam: Richtungsvektor

        # Geradengleichung 2
        P2 = self.camsData[1].transVecGlobal
        R2 = np.array(np.add(self.rayCam2.Pbe,-P2))  # Object <-> Cam: Richtungsvektor

        z = np.array(P2-P1)
        r = R1
        s = R2

        m = np.column_stack((s,r,z))

        z1 = np.sum(np.multiply(z,R1))
        r1 = np.sum(np.multiply(r,R1))
        s1 = np.sum(np.multiply(-s,R1))

        z2 = np.sum(np.multiply(z,R2))
        r2 = np.sum(np.multiply(r,R2))
        s2 = np.sum(np.multiply(-s,R2))

        # solving
        a = np.array([[r1, s1], [r2, s2]], dtype="double")
        b = np.array([-z1, -z2], dtype="double")

        # Solve linear equation
        # invA = np.linalg.inv(a)
        invA = np.linalg.pinv(a)
        res = np.dot(invA,b)

        # Calculate Pg1 and Pg2. Actual point on lines in 3D space
        Pg1 = np.add(P1, np.multiply(-res[0],R1))
        Pg2 = np.add(P2, np.multiply(-res[1],R2))
        Pcenter = Pg1 + 0.5*(Pg2-Pg1)

        #Calculate Distance from cam to object  and between lines
        dist_cam1_object = np.linalg.norm(Pcenter-P1)
        dist_cam2_object = np.linalg.norm(Pcenter-P2)
        dist_line = np.linalg.norm(Pg2-Pg1)

        ## Add attributes
        self.P1 = P1
        self.P2 = P2
        self.R1 = R1
        self.R2 = R2
        self.Pg1 = Pg1
        self.Pg2 = Pg2
        self.dist_cam1_object = dist_cam1_object
        self.dist_cam2_object = dist_cam2_object
        self.dist_line = dist_line
        self.Pcenter = Pcenter

### Global methods
def importSettings():
    """Import Settings """
    with open('src/config.json', 'r') as f:
        config = json.load(f)
    return(config)

def timePrint(start,topic,count):
    print(topic, " finished after: ", (time.time() - start)*1000, " ms", "n_intersections: ", count)

def consumeKafka(calibrationData, config, categories):
    """KafkaConsumer, directly connected to KafkaCluster on 10.42.0.1

    This is Kafka consumer and producer. Receiving messages directly our of
    active DeepStream Pipelines on CameraDevices. Producer will send intersection
    results for further visualization.
    """
    global puffCam1
    global puffData
    global puffCam2
    consumer = KafkaConsumer('deepstream',bootstrap_servers=['10.42.0.1:9092'])
    producer = KafkaProducer(bootstrap_servers="10.42.0.1:9092", value_serializer=lambda v: json.dumps(v).encode('utf-8'))

    ### start logging Kafka Realttime Stream to file Consumer and Producer
    name = str(datetime.datetime.now())
    c = open("log/consumer_" + name + ".json", "w")
    p = open("log/producer_" + name + ".json", "w")
    data_file = open("log/data_" + name + ".json", "w")

    ### Calculate for every new KafkaConsumer Message and prduce results.
    for message in consumer:
        ### Log consumers kafka stream to file
        c.write(str(json.loads(message.value))+"\n")

        ### Intersections calculations, based on json payload
        jsonMessage = json.loads(message.value)

        start = time.time()

        temp = []
        tempdata = []
        # Jedes Objekt durchlaufen und den Ray berechnen
        for object in jsonMessage['objects']:
            # für jedes object nen ray berechnen
            pos = object.split('|')
            # use other calibration for cam 1 and 2
            if (jsonMessage['sensorId'] == 'Camera1'):
                temp.append(Ray(pos, camsData[0]))
                tempdata.append(object)
            else:
                temp.append(Ray(pos, camsData[1]))

        # Wenn Objekte gerade von Camera 1 kommen dies überschreiben
        if (jsonMessage['sensorId'] == 'Camera1'):
            puffCam1 = []
            puffCam1 = temp
            puffData = []
            puffData = tempdata
        else:
            puffCam2 = []
            puffCam2 = temp

        # Do intersection calculation between newest rays and all objects which are inside!
        intersections = []
        # for every ray in puffCam1!
        for rayCam1 in puffCam1:
            for rayCam2 in puffCam2:
                intersection = Intersection(rayCam1, rayCam2, camsData)
                # add to Frame-Intersections
                intersections.append(intersection)

        for object in intersections:
            # Define Max Distance of intersection
            if (object.dist_line < 0.7):
                # print("Distance:", object.dist_line) # log

                d = {
                    'posCam1': '[' + str(object.camsData[0].transVecGlobal[0][0]) + ',' + str(object.camsData[0].transVecGlobal[1][0]) + ',' + str(object.camsData[0].transVecGlobal[2][0]) + ']',
                    'posCam2': '[' + str(object.camsData[1].transVecGlobal[0][0]) + ',' + str(object.camsData[1].transVecGlobal[1][0]) + ',' + str(object.camsData[1].transVecGlobal[2][0]) + ']',
                    'Pg1': '[' + str(object.Pg1[0][0]) + ',' + str(object.Pg1[1][0]) + ',' + str(object.Pg1[2][0]) + ']',
                    'Pg2': '[' + str(object.Pg2[0][0]) + ',' + str(object.Pg2[1][0]) + ',' + str(object.Pg2[2][0]) + ']',
                    'Pcenter': '[' + str(object.Pcenter[0][0]) + ',' + str(object.Pcenter[1][0]) + ',' + str(object.Pcenter[2][0]) + ']'
                }

                data_file.write(str(d)+"\n")
                producer.send("detection_details", d)

                ### Log producers kafka stream to file
                p.write(str(d)+"\n")

        timePrint(start,'Frame calculation time: ',len(intersections))

### Self init
if __name__ == "__main__":
    ### Define global vars
    global puffCam1, puffCam2 , a
    puffCam1 = []
    puffCam2 = []
    categories = 'Person|Car'
    dataPath = "src/config.json"

    ### Handle args =>
    # [1] localCalibrationData -> local, cams
    # [2] calibrationMethod -> aruco
    try:
        calibrationMethod = sys.argv[2]
        print("Calibration Method: ",calibrationMethod)
    except:
         print("Pls select calibration method!")
         sys.exit(0)
    try:
        CalibrationDataOrigin = sys.argv[1]
        print("Calibration Data Origin: ",CalibrationDataOrigin)
    except:
        print("Please select calibration data origin!")
        sys.exit(0)

    ### if everything is fine print it out!
    print("Ready for kafka Stream Input from cams!")
    print("Waiting......")

    ### Load config
    config = importSettings()

    ### Create Cam Objects
    cam1 = Camera("cam1", dataPath, CalibrationDataOrigin, calibrationMethod)
    cam2 = Camera("cam2", dataPath, CalibrationDataOrigin, calibrationMethod)
    camsData = [cam1, cam2]

    # RealTime Calculation of intersections with use of KafkaConsumer
    consumeKafka(camsData, config, categories)
