#coding=utf-8
import rospy
from std_msgs.msg import Header
from sensor_msgs.msg import Image, NavSatFix
from map_generator.msg import tjy
from nav_msgs.msg import Path
import numpy as np
import time
from googleplaces import GooglePlaces
import googlemaps
import time
import sys
import math
from math import cos,sin,tan,sqrt
from visualization_msgs.msg import Marker
sys.path.remove('/opt/ros/kinetic/lib/python2.7/dist-packages')
C_EARTH = 6378137.0

class GoogleMaps(object):

    def __init__(self):

        self._GOOGLE_MAPS_KEY = "AIzaSyD4gxyxClNkmTXbTkTseNvFwA16YR9NAPE"
        self._Google_Places = GooglePlaces(self._GOOGLE_MAPS_KEY)
        self._Google_Geocod = googlemaps.Client(key=self._GOOGLE_MAPS_KEY)

    def _nearby_search(self, lng, lat,  language, radius, result=None):
        if result is None:
            nearby_query_result = self._Google_Places.nearby_search(language=language,
                lat_lng={'lat': lat, 'lng': lng}, radius=radius)
        else:
            if result.has_next_page_token:
                #print(result.next_page_token)
                nearby_query_result = self._Google_Places.nearby_search(
                    pagetoken=result.next_page_token, lat_lng={'lat': lat, 'lng': lng}, radius=radius)
            else:
                nearby_query_result = None

        return nearby_query_result

    def get_all_data(self, lng, lat,  language='en', radius=100):
        count = 0
        list_return_info = []
        
        list_nearby_search_result = self._nearby_search(lng, lat,  language, radius)
        while(list_nearby_search_result is not None):
            for place in list_nearby_search_result.places:
                # Returned places from a query are place summaries.
                print(place.name)
                print(place.geo_location['lng'])
                print(place.geo_location['lat'])
                print(count)
                count = count+1
                list_return_info.append({"name":place.name, "lng":place.geo_location['lng'], "lat":place.geo_location['lat']})
                
                #print place.place_id

                # The following method has to make a further API call.
                #place.get_details()
                # Referencing any of the attributes below, prior to making a call to
                # get_details() will raise a googleplaces.GooglePlacesAttributeError.
                #print place.details  # A dict matching the JSON response from Google.
                #print place.local_phone_number
                #print place.international_phone_number
                #print place.website
                #print place.url

            # Are there any additional pages of results?
            list_nearby_search_result = self._nearby_search(lng, lat,  language, radius, list_nearby_search_result)
        
        return list_return_info

class Transform(object):
    def __init__(self):
        self.R = None
        self.t = None

    def centroid_point(self, samples):
        means = np.mean(samples, axis=0)
        return means

    def transform_lamda(self, A, B):
        A_norm = np.sum(A*A,axis=1)
        B_norm = np.sum(B*B,axis=1)

        #lam=np.sqrt(A_norm)/np.sqrt(B_norm)
        lam = A_norm/B_norm
        lam=np.mean(lam)
        return lam
    
    def transform_3D_RT(self, A, B):
        
        assert A.shape == B.shape
        # A is original, B is target
        centroidA = self.centroid_point(A)
        centroidB = self.centroid_point(B)

        H = np.dot((A - centroidA).T , (B - centroidB))

        A_move=A - centroidA
        B_move=B - centroidB

        lam = self.transform_lamda(A_move, B_move)

        U,S,V = np.linalg.svd(H)
        R = np.dot(V,U.T)

        if np.linalg.det(R) < 0:
            #print('Reflection detected')
            V[:,2] = -1*V[:,2]
            R = np.dot(V,U.T)

        t = - np.dot((R/sqrt(lam)),centroidA.T) + centroidB.T
        R = R/sqrt(lam)
        self.R= R
        self.t = t.reshape((3,1))


        return R, t

    def transform(self, A, R = None, t = None):
        if R is None:
            R = self.R
            t = self.t
        B = np.dot(R, A.T) + t
        return B

        


class NearbySearch(object):
    def __init__(self):
        self._sub = rospy.Subscriber('/trajectory',tjy, self.callback, queue_size=100)
        #self._pub = rospy.Publisher('/nearby_gps', NavSatFix, queue_size = 100)
        #self._pub1 = rospy.Publisher('/car_gps', NavSatFix, queue_size = 100)
        self._pub = rospy.Publisher('/location', Marker, queue_size=1000)
        self.google_maps = GoogleMaps()
        self.count = 0
        self.gps_result = []
        self.new_gps = []
        #self.xyz_temp = NavSatFix()
        self._timenum = 0
        self.init_lat =0.0
        self.init_lng = 0.0
        self.init_x = 0.0
        self.init_y = 0.0
        self.init_z = 0.0
        self.init = True
        self.init_pose_x = 0.0
        self.init_pose_y = 0.0
        self.init_pose_z = 0.0

        self.number_limit = 20
        self.sample_num = 30
        self.xyz_samples = []
        self.pose_samples = []
        self.ave_xyz_samples = []
        self.ave_pose_samples = []
        self.transform = Transform()

        self.display_freq = 10
        self.marker_scale = 0.2
        self.marker_lifetime = 8 # 0 is forever
        self.marker_id = 0
        self.marker_ns = 'building'+str(self.marker_id) 
        self.marker_color = {'r': 1.0, 'g': 0.7, 'b': 1.0, 'a': 1.0}
        #self.marker_init()

    def add_point(self, samples, point):
        if len(samples)== self.number_limit:
            samples.remove(samples[0])
        samples.append(point)
        return samples

    def ave_append(self, ave, sample):
        if len(sample) == self.number_limit:
            sam = np.mean(np.array(sample).reshape((-1,3)), axis=0)
            ave.append([sam[0], sam[1], sam[2]])
        return ave

    def add_samples(self, samples, ave_samples):
        if not len(ave_samples) == 0:
            new = ave_samples + samples
        else:
            new = samples
        print("current groundtrue sample: ", len(new))
        return np.array(new).reshape((-1,3))

    def marker_init(self, markers):
        self.marker_ns = 'building'+str(self.marker_id) 
        markers.ns = self.marker_ns
        markers.id = self.marker_id
        markers.type = Marker.CUBE
        markers.action = Marker.ADD
        markers.lifetime = rospy.Duration(self.marker_lifetime)
        markers.scale.x = self.marker_scale
        markers.scale.y = self.marker_scale
        markers.scale.z = self.marker_scale
        markers.color.r = self.marker_color['r']
        markers.color.g = self.marker_color['g']
        markers.color.b = self.marker_color['b']
        markers.color.a = self.marker_color['a']
        markers.header.frame_id = "sensor_frame"
        markers.header.stamp = rospy.Time.now()
        markers.frame_locked = True
        #markers.points = list()
        return markers

    def name_init(self, markers):
        self.marker_ns = 'name'+str(self.marker_id+1) 
        markers.ns = self.marker_ns
        markers.id = self.marker_id+1
        markers.type = Marker.TEXT_VIEW_FACING
        markers.action = Marker.ADD
        markers.lifetime = rospy.Duration(self.marker_lifetime)
        markers.scale.x = self.marker_scale
        markers.scale.y = self.marker_scale
        markers.scale.z = self.marker_scale
        markers.color.r = self.marker_color['r']
        markers.color.g = self.marker_color['g'] - 0.4
        markers.color.b = self.marker_color['b']
        markers.color.a = self.marker_color['a']
        markers.header.frame_id = "sensor_frame"
        markers.header.stamp = rospy.Time.now()
        markers.frame_locked = True
        #markers.points = list()
        return markers


    def combine(self, result, new):
        newgps = []
        for line in new:
            if not line in result:
                result.append(line)
                newgps.append(line)
        print("current result: ",len(result))
        return result, newgps
    '''
    def gps_xyz_convert(self, lat, lng, alt=0.0):
        deltalat = lat - self.init_lat
        deltalng = lng - self.init_lng
        deltax = deltalat * C_EARTH
        deltay = deltalng * C_EARTH * cos(lat)
        return x,y,z
    '''
    def gps_xyz_convert(self, lat, lng, alt=0.0):
        Datum=84        #投影基准面类型：北京54基准面为54，西安80基准面为80，WGS84基准面为84
        prjno=0       #投影带号
        zonewide=3        
        IPI=0.0174532925199433333333        #3.1415926535898/180.0
        B=lat #纬度
        L=lng #经度
        if zonewide==6:
            prjno=(int)(L/zonewide)+1
            L0=prjno*zonewide-3
        else:
            prjno=(int)((L-1.5)/3)+1
            L0=prjno*3
        
        if(Datum==54):
            a=6378245
            f=1/298.3 
        elif(Datum==84):
            a=6378137
            f=1/298.257223563
        L0=L0*IPI
        L=L*IPI
        B=B*IPI
 
        e2=2*f-f*f
        l=L-L0
        t=tan(B)
        m=l * cos(B)
        N=a/sqrt(1-e2* sin(B) * sin(B))
        q2=e2/(1-e2)* cos(B)* cos(B)
        a1=1+float(3/4)*e2+float(45/64)*e2*e2+float(175/256)*e2*e2*e2+float(11025/16384)*e2*e2*e2*e2+float(43659/65536)*e2*e2*e2*e2*e2
        a2=float(3/4)*e2+float(15/16)*e2*e2+float(525/512)*e2*e2*e2+float(2205/2048)*e2*e2*e2*e2+float(72765/65536)*e2*e2*e2*e2*e2
        a3=float(15/64)*e2*e2+float(105/256)*e2*e2*e2+float(2205/4096)*e2*e2*e2*e2+float(10359/16384)*e2*e2*e2*e2*e2
        a4=float(35/512)*e2*e2*e2+float(315/2048)*e2*e2*e2*e2+float(31185/13072)*e2*e2*e2*e2*e2
        b1=a1*a*(1-e2)
        b2=float(-1/2)*a2*a*(1-e2)
        b3=float(1/4)*a3*a*(1-e2)
        b4=float(-1/6)*a4*a*(1-e2)
        c0=b1
        c1=2*b2+4*b3+6*b4
        c2=-(8*b3+32*b4)
        c3=32*b4
        s=c0*B+cos(B)*(c1*sin(B)+c2*sin(B)*sin(B)*sin(B)+c3*sin(B)*sin(B)*sin(B)*sin(B)*sin(B))
        x=s+float(1/2)*N*t*m*m+float(1/24)*(5-t*t+9*q2+4*q2*q2)*N*t*m*m*m*m+float(1/720)*(61-58*t*t+t*t*t*t)*N*t*m*m*m*m*m*m
        y=N*m+float(1/6)*(1-t*t+q2)*N*m*m*m+float(1/120)*(5-18*t*t+t*t*t*t-14*q2-58*q2*t*t)*N*m*m*m*m*m
        y=y+1000000*prjno+500000

        return x, y-38000000, alt


    def callback(self, msg):
        currentpose = msg.tjy.poses[-1]
        currentgps = msg.gps[-1]
        lat = currentgps.latitude
        lng = currentgps.longitude
        alt = currentgps.altitude
        if self.init:
            self.init_lat = lat
            self.init_lng = lng
            self.init_x, self.init_y, self.init_z = self.gps_xyz_convert(lat, lng)
            #self.init_x, self.init_y, self.init_z = lat, lng, alt
            print("init xyz ", self.init_x, self.init_y, self.init_z)

            self.init_pose_x = currentpose.pose.position.x
            self.init_pose_y = currentpose.pose.position.y
            self.init_pose_z = currentpose.pose.position.z
            print("init pose ", self.init_pose_x, self.init_pose_y, self.init_pose_z)

            self.xyz_samples = self.add_point(self.xyz_samples,[0,0,0])
            #self.pose_samples = np.array([self.init_pose_x, self.init_pose_y, self.init_pose_z]).reshape((1,3))
            self.pose_samples = self.add_point(self.pose_samples, [0, 0, 0])
            self.init = False

        else:
            temp_x, temp_y, temp_z = self.gps_xyz_convert(lat, lng)
            #temp_x, temp_y, temp_z = lat, lng, alt
            temp_x = temp_x - self.init_x
            temp_y = temp_y - self.init_y
            temp_z = temp_z - self.init_z
            self.xyz_samples = self.add_point(self.xyz_samples,[temp_x, temp_y, temp_z])
            #self.xyz_samples = np.concatenate((self.xyz_samples , np.array([temp_x, temp_y, temp_z]).reshape((1,3))),axis=0)
            temp_pose_x = currentpose.pose.position.x
            temp_pose_y = currentpose.pose.position.y
            temp_pose_z = currentpose.pose.position.z
            #self.pose_samples = np.concatenate((self.pose_samples, np.array([temp_pose_x, temp_pose_y, temp_pose_z]).reshape((1,3))), axis=0)
            #self.pose_samples = np.concatenate((self.pose_samples, np.array([temp_pose_x, temp_pose_y, 0.0]).reshape((1,3))), axis=0)
            self.pose_samples = self.add_point(self.pose_samples, [temp_pose_z, temp_pose_x, temp_pose_y])

            
        if self._timenum%self.sample_num == 0:
            self.ave_pose_samples = self.ave_append(self.ave_pose_samples, self.pose_samples)
            self.ave_xyz_samples = self.ave_append(self.ave_xyz_samples, self.xyz_samples)

        if self._timenum%self.display_freq == 1:
            print("latitude: {0}, longitude: {1}, altitude: {2}".format(lat, lng, alt))
            
            #list_return_info = self.google_maps.get_all_data(lng, lat)
            list_return_info = temp_read()
            print("find gps info")
            self.gps_result ,self.new_gps = self.combine(self.gps_result, list_return_info)
            #if not len(self.new_gps)==0:
                #self.count = len(self.gps_result)
            
            #R, t = self.transform.transform_3D_RT(self.xyz_samples, self.pose_samples)
            R, t = self.transform.transform_3D_RT(self.add_samples(self.xyz_samples, self.ave_xyz_samples), self.add_samples(self.pose_samples, self.ave_pose_samples))
            #R, t = self.transform.transform_3D_RT(np.array([[0,0,0],[temp_x, temp_y, temp_z]]).reshape((-1,3)), np.array([[0,0,0],[temp_pose_z, temp_pose_x, temp_pose_y]]).reshape((-1,3)))
            print("R ",R)
            print("t ",t)

            temp_pose = self.transform.transform(np.array([temp_x, temp_y, temp_z]).reshape((1,3)))
            #print("temp_pose: ",temp_pose)
            #print("true pose: ",np.array([temp_pose_z, temp_pose_x, temp_pose_y]).reshape((3,1)))
            print("distance",self.distance(temp_pose.reshape((3,1)), np.array([temp_pose_z, temp_pose_x, temp_pose_y]).reshape((3,1))))
            for point in self.gps_result:
                name = point["name"]
                gps_lat = point["lat"]
                gps_lng = point["lng"]
                ros_x, ros_y, ros_z = self.gps_xyz_convert(gps_lat, gps_lng)
                #ros_x, ros_y, ros_z = gps_lat, gps_lng,alt
                ros_x = ros_x - self.init_x
                ros_y = ros_y - self.init_y
                ros_z = ros_z - self.init_z
                #print("name: {0}, latitude: {1}, longitude: {2}, altitude: {3}".format(name, gps_lat, gps_lng, alt))

                ros_pose = self.transform.transform(np.array([ros_x,ros_y,ros_z]).reshape((1,3)))

                #print("ros_pose", ros_pose)

                self.marker_id = self.count
                self.count += 2
                marker = Marker()
                namemarker = Marker()
                marker = self.marker_init(marker)
                namemarker = self.name_init(namemarker)
                namemarker.text = name

                marker.pose.position.x = ros_pose[1]
                marker.pose.position.y = ros_pose[2]
                marker.pose.position.z = ros_pose[0]
                marker.pose.orientation.y = 0.0
                marker.pose.orientation.z = 0.0
                marker.pose.orientation.w = 1.0
                self._pub.publish(marker)

                namemarker.pose.position.x = ros_pose[1]
                namemarker.pose.position.y = ros_pose[2] - 0.3
                namemarker.pose.position.z = ros_pose[0]
                namemarker.pose.orientation.y = 0.0
                namemarker.pose.orientation.z = 0.0
                namemarker.pose.orientation.w = 1.0
                self._pub.publish(namemarker)




                self.new_gps = []

        self._timenum = self._timenum + 1

    def distance(self, ros, pose):
        dis = np.sqrt(np.sum((ros.reshape((3,1))-pose.reshape((3,1)))**2))
        return dis

    def main(self):
        rospy.spin()
        def save_xyz():
            f = open("building_xyz.txt", "w")
            
            for point in self.gps_result:
                name = point["name"]
                gps_lat = point["lat"]
                gps_lng = point["lng"]
                ros_x, ros_y, ros_z = self.gps_xyz_convert(gps_lat, gps_lng)
                ros_x = ros_x - self.init_x
                ros_y = ros_y - self.init_y
                ros_z = ros_z - self.init_z

                ros_pose = self.transform.transform(np.array([ros_x,ros_y,ros_z]).reshape((1,3)))

                f.write(name+";%f" %ros_pose[1]+";%f" %ros_pose[2]+";%f" %ros_pose[0]+"\n")
            f.close()
        rospy.on_shutdown(save_xyz)

def temp_read():
    f = open("../../05/info.txt","r")
    data = []
    for line in f.readlines():
        line = line.strip('\n')
        line = line.strip('\r')
        line = line.split(',')
        if " " in line:
            line.remove(" ")
        data.append({"name":line[0], "lng":float(line[1]), "lat":float(line[2])})
    f.close()
    return data

    


if __name__ == '__main__':
    
    rospy.init_node("NearbySearch", anonymous=True)
    nearby = NearbySearch()
    nearby.main()
    '''
    transform = Transform()
    a = np.array([[0,0,0],[1.1,5.6,0],[2.5,3.2,0],[6.9,8.4,0]]).reshape((-1,3))
    b = np.array([[0,0,0],[0.11, 0.56,0.0],[0.25,0.32,0],[0.69,0.84,0]]).reshape((-1,3))
    R, t = transform.transform_3D_RT(a,b)
    m = np.array([1.1,5.6,0]).reshape((1,3))
    n = np.array([10.1, 11.6, 0]).reshape((1,3))
    mb = transform.transform(m)
    nb = transform.transform(n)
    print(mb)
    print(nb)
    '''
    

