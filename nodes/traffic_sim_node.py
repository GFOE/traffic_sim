#!/usr/bin/env python

import rospy
from geographic_msgs.msg import GeoPointStamped
from marine_msgs.msg import Contact
import project11
import random
import math

asv_position = None

vessels = []

target_vessel_count = 50

new_vessel_range = (500,5000)
vessel_delete_range = 20000
vessel_speed_range = (2,10) # m/s
next_id = 1

contact_pub = None

class Vessel:
    def __init__(self,start_time):
        global next_id
        self.last_time = start_time
        distance =  random.uniform(new_vessel_range[0],new_vessel_range[1])
        bearing = random.uniform(0,360)
        self.position = project11.geodesic.direct(math.radians(asv_position.position.longitude),math.radians(asv_position.position.latitude),math.radians(bearing),distance)
        self.heading = math.radians((bearing+180+random.uniform(-90,90))%360)
        self.last_report_time = start_time + rospy.Duration.from_sec(random.uniform(-10,0.0))
        self.target_id = 'sim{:04}'.format(next_id)
        self.mmsi = next_id
        next_id += 1
        
        self.speed = random.uniform(vessel_speed_range[0],vessel_speed_range[1])
    
    def iterate(self, data):
        delta_t = data.current_real - self.last_time
        if delta_t > rospy.Duration(0):
            distance = delta_t.to_sec()*self.speed
            self.position = project11.geodesic.direct(self.position[0],self.position[1],self.heading,distance)
            self.last_time = data.current_real
        
        if self.last_report_time is None or data.current_real - self.last_report_time > rospy.Duration.from_sec(10.0):
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
            
            contact.callsign = self.target_id
            
            contact_pub.publish(contact)
            
            self.last_report_time = data.current_real
    
def iterate(data):
    for v in vessels:
        v.iterate(data)
    
    if asv_position is not None and not data.current_real.is_zero():
        while len(vessels) < target_vessel_count:
            v = Vessel(data.current_real)
            vessels.append(v)
        

def positionCallback(data):
    global asv_position
    asv_position = data

if __name__ == '__main__':
    rospy.init_node('traffic_sim')
    
    contact_pub = rospy.Publisher('/contact',Contact,queue_size = 10)
    
    rospy.Subscriber('/position', GeoPointStamped, positionCallback)
    rospy.Timer(rospy.Duration.from_sec(0.2),iterate)
    rospy.spin()


    
