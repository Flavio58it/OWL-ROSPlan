#!/usr/bin/env python

import sys
import rospy

from std_msgs.msg import *
from diagnostic_msgs.msg import KeyValue
from rosplan_dispatch_msgs.msg import *
from rosplan_knowledge_msgs.srv import *
from rosplan_knowledge_msgs.msg import *

from armor_api.armor_client import ArmorClient

class ArmorManager():
    def __init__(self, client_id, reference_name):
        self.client = ArmorClient("KCL_rosplan","robot_explore")
        self.client_id = client_id
        self.reference_name = reference_name

    # Manipulations
    def add_dataprop_to_ind(self, prop_name, ind_name, data_type, value):
        self.client.call("ADD", "DATAPROP", "IND", [prop_name, ind_name, data_type, value])

    def add_ind_to_cls(self, ind_name, cls_name):
        self.client.call("ADD", "IND", "CLASS", [ind_name, cls_name])

    def remove_ind(self, ind_name):    
        res = self.client.call("REMOVE", "IND", "", [ind_name])
        return res

    # Queries
    def query_dataprop_b2_ind(self, prop_name, ind_name):
        res = self.client.call("QUERY", "DATAPROP", "IND", [prop_name, ind_name])    
        return res.queried_objects

    # 
    def apply_changes(self):
        if self.client.call("DISJOINT", "IND", "CLASS", ["Item"])==False:
            return False
        if self.client.call("APPLY", "", "", [])==False:
            return False
        res = self.client.call("REASON", "", "", [])
        return res



if __name__ == "__main__":
    
    client_id= "KCL_rosplan"
    reference_name = "robot_explore"
    armor =  ArmorManager(client_id, reference_name)
