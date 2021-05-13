#!/usr/bin/env python

from __future__ import print_function
from builtins import str
from builtins import range
from builtins import object
import rospy
from geographic_msgs.msg import GeoPointStamped
from nav_msgs.msg import Odometry
from marine_msgs.msg import Contact
from sensor_msgs.msg import Joy
import project11
import random
import math
import tf2_ros
from tf2_geometry_msgs import do_transform_pose

asv_position = None

vessels = []
joy_vessels = {}

target_vessel_count = 5

new_vessel_range = (250,1000)
vessel_delete_range = 3000
vessel_speed_range = (2,10) # m/s
next_id = 1

contact_pub = None


class Vessel(object):
    def __init__(self,start_time=None, joy_topic=None):
        global next_id
        self.last_time = start_time
        if joy_topic is None:
            distance =  random.uniform(new_vessel_range[0],new_vessel_range[1])
            bearing = random.uniform(0,360)
        else:
            distance = 100
            joy_id = int(joy_topic[-1])
            print('joy_id',joy_id)
            bearing = -45.0 + 90*joy_id
                
        self.position = project11.geodesic.direct(asv_position[1],asv_position[0],math.radians(bearing),distance)
        
        # itinitate a random heading that generally points towards the vehicle
        self.heading = math.radians((bearing+180+random.uniform(-90,90))%360)
        self.rate_of_turn = 0.0
        
        # stagger the reporting via AIS by randomly assigning a last report time in the last 10 seconds.
        if start_time is None:
            self.last_report_time = None
        else:
            self.last_report_time = start_time + rospy.Duration.from_sec(random.uniform(-10.0,0.0))
        

        if joy_topic is not None:
            rospy.Subscriber(joy_topic, Joy, self.joystickCallback)
            self.target_id = joy_topic
            self.report_rate = 0.25
        else:
            self.target_id = 'sim{:04}'.format(next_id)
            self.report_rate = 10.0

        self.mmsi = next_id
        next_id += 1
        
        self.speed = random.uniform(vessel_speed_range[0],vessel_speed_range[1])
        
        self.throttle = None
        self.rudder = None
        
        self.max_speed = 8.0
        self.min_speed = 0.0
        self.speed_change_rate = 1.5
        
        self.dims = {'stbd':random.uniform(1,3),
                     'port':random.uniform(1,3),
                     'bow':random.uniform(2,5),
                     'stern':random.uniform(3,8)}
    
    def iterate(self, data):
        if self.last_time is not None:
            delta_t = data.current_real - self.last_time
            if delta_t > rospy.Duration(0):
                if self.throttle is not None:
                    target_speed = max(self.min_speed,self.throttle*self.max_speed)
                    if target_speed > self.speed:
                        delta_speed = self.speed_change_rate*delta_t.to_sec()
                        self.speed = min(self.speed+delta_speed,target_speed)
                    else:
                        delta_speed = -self.speed_change_rate*delta_t.to_sec()
                        self.speed = max(self.speed+delta_speed,target_speed)
                if self.rudder is not None:
                    self.rate_of_turn = self.rudder
                else:
                    if random.randrange(1000) == 0:
                        self.heading = math.radians((math.degrees(self.heading)+random.uniform(-135,135))%360)
                        self.last_report_time = None



                        
                distance = delta_t.to_sec()*self.speed
                self.heading += self.rate_of_turn*delta_t.to_sec()
                self.position = project11.geodesic.direct(self.position[0],self.position[1],self.heading,distance)
        self.last_time = data.current_real
        
        if self.last_report_time is None or data.current_real - self.last_report_time > rospy.Duration.from_sec(self.report_rate):
            contact = Contact()
            contact.header.stamp = data.current_real
            contact.contact_source = Contact.CONTACT_SOURCE_AIS
            contact.mmsi = self.mmsi
            contact.name = 'Vessel known as '+self.target_id
            contact.position.latitude = math.degrees(self.position[1])
            contact.position.longitude = math.degrees(self.position[0])
            contact.heading = self.heading
            contact.cog = self.heading
            contact.sog = self.speed
            
            contact.dimension_to_stbd = self.dims['stbd']
            contact.dimension_to_port = self.dims['port']
            contact.dimension_to_bow = self.dims['bow']
            contact.dimension_to_stern = self.dims['stern']
            
            contact.callsign = self.target_id
            
            contact_pub.publish(contact)
            
            self.last_report_time = data.current_real
            
    def distance(self):
        b,d = project11.geodesic.inverse(asv_position[1],asv_position[0],self.position[0],self.position[1])
        return d

    def joystickCallback(self,data):
        if len(data.axes) >= 4 and len(data.buttons) >= 9:
            self.throttle = data.axes[1]
            self.rudder = -data.axes[3]
            if data.buttons[8]:
                self.throttle = 0.0
                self.rudder = 0.0
                
                distance = 100
                joy_id = int(self.target_id[-1])
                bearing = -45.0 + 90*joy_id
                    
                self.position = project11.geodesic.direct(asv_position[1],asv_position[0],math.radians(bearing),distance)
                self.heading = math.radians(bearing)

            


def iterate(data):
    to_delete = []
    for v in vessels:
        if v.distance() > vessel_delete_range:
            to_delete.append(v)
    for v in to_delete:
        vessels.remove(v)
    for v in vessels:
        v.iterate(data)
    for v in list(joy_vessels.values()):
        v.iterate(data)
    
    if asv_position is not None and not data.current_real.is_zero():
        while len(vessels) < target_vessel_count:
            v = Vessel(data.current_real)
            vessels.append(v)
        if len(joy_vessels) < 4:
            for i in range(4):
                topic = '/joy_'+str(i)
                joy_vessels[topic] = Vessel(None,topic)
                
        

def odomCallback(data):
    global asv_position
    try:
        odom_to_earth = tfBuffer.lookup_transform("earth", data.header.frame_id, rospy.Time(0.0), rospy.Duration(0.5))
        ecef = do_transform_pose(data.pose, odom_to_earth).pose.position
        asv_position = project11.wgs84.fromECEFtoLatLong(ecef.x, ecef.y, ecef.z)
    except Exception as e:
        print(e)
        

if __name__ == '__main__':
    rospy.init_node('traffic_sim')
    
    tfBuffer = tf2_ros.Buffer()
    tfListener = tf2_ros.TransformListener(tfBuffer)

    contact_pub = rospy.Publisher('sensors/ais/contact',Contact,queue_size = 10)
    
    rospy.Subscriber('odom', Odometry, odomCallback)
    
    rospy.Timer(rospy.Duration.from_sec(0.2),iterate)
    rospy.spin()


    
