#!/usr/bin/env python

import sys
import rospy

import rosplan_knowledge_msgs.srv
import rosplan_knowledge_msgs.msg
import diagnostic_msgs.msg

from armor_manager import ArmorManager


def add_instance_call(service, name, type):
    msg = rosplan_knowledge_msgs.srv.KnowledgeUpdateServiceRequest()
    knowledge_item = rosplan_knowledge_msgs.msg.KnowledgeItem()
    msg.update_type = msg.ADD_KNOWLEDGE
    knowledge_item.knowledge_type = knowledge_item.INSTANCE
    knowledge_item.instance_type = type
    knowledge_item.instance_name = name
    msg.knowledge = knowledge_item
    service.call(msg)
    return


def add_fact_call(service, attribute, args):
    msg = rosplan_knowledge_msgs.srv.KnowledgeUpdateServiceRequest()
    knowledge_item = rosplan_knowledge_msgs.msg.KnowledgeItem()
    msg.update_type = msg.ADD_KNOWLEDGE
    knowledge_item.knowledge_type = knowledge_item.FACT
    knowledge_item.attribute_name = attribute

    for i, arg in enumerate(args):
        key_value = diagnostic_msgs.msg.KeyValue()
        key_value.key = "has_arg_" + str(i + 1)
        key_value.value = arg
        knowledge_item.values.append(key_value)

    msg.knowledge = knowledge_item
    service.call(msg)
    return


def add_goal_call(service, attribute, args):
    msg = rosplan_knowledge_msgs.srv.KnowledgeUpdateServiceRequest()
    knowledge_item = rosplan_knowledge_msgs.msg.KnowledgeItem()
    msg.update_type = msg.ADD_GOAL
    knowledge_item.knowledge_type = knowledge_item.FACT
    knowledge_item.attribute_name = attribute

    for i, arg in enumerate(args):
        key_value = diagnostic_msgs.msg.KeyValue()
        key_value.key = "has_arg_" + str(i + 1)
        key_value.value = arg
        knowledge_item.values.append(key_value)

    msg.knowledge = knowledge_item
    service.call(msg)
    return

def remove_instance_call(service, name, type):
    msg = rosplan_knowledge_msgs.srv.KnowledgeUpdateServiceRequest()
    knowledge_item = rosplan_knowledge_msgs.msg.KnowledgeItem()
    msg.update_type = msg.REMOVE_KNOWLEDGE
    knowledge_item.knowledge_type = knowledge_item.INSTANCE
    knowledge_item.instance_type = type
    knowledge_item.instance_name = name
    msg.knowledge = knowledge_item
    service.call(msg)
    return

def initialize_knowledge_client():
    rospy.wait_for_service('/kcl_rosplan/update_knowledge_base')
    try:
        service = rospy.ServiceProxy('/kcl_rosplan/update_knowledge_base',
                                     rosplan_knowledge_msgs.srv.KnowledgeUpdateService)
        return service
    except rospy.ServiceException, e:
        print "Service call failed: %s" % e

def recognize(service):
    # add instances
    add_instance_call(service, "wp1", "waypoint")
    add_instance_call(service, "wp2", "waypoint")
    add_instance_call(service, "wp3", "waypoint")
    add_instance_call(service, "wp4", "waypoint")
    add_instance_call(service, "wp5", "waypoint")
    add_instance_call(service, "wp6", "waypoint")
    add_instance_call(service, "mobile_robot", "robot")

    add_instance_call(service, "milk", "object")
    add_instance_call(service, "ball", "object")

    # add attributes
    add_fact_call(service, "robot_at", ["mobile_robot", "wp1"])
    add_fact_call(service, "connected", ["wp1", "wp2"]), add_fact_call(service, "connected", ["wp2", "wp1"])
    add_fact_call(service, "connected", ["wp2", "wp3"]), add_fact_call(service, "connected", ["wp3", "wp2"])
    add_fact_call(service, "connected", ["wp4", "wp3"]), add_fact_call(service, "connected", ["wp3", "wp4"])
    add_fact_call(service, "connected", ["wp5", "wp2"]), add_fact_call(service, "connected", ["wp2", "wp5"])
    add_fact_call(service, "connected", ["wp6", "wp5"]), add_fact_call(service, "connected", ["wp5", "wp6"])
    add_fact_call(service, "visible", ["mobile_robot", "wp6", "milk"]) 
    add_fact_call(service, "visible", ["mobile_robot", "wp4", "ball"])
    
    # add goals
    add_goal_call(service, "detected", ["milk"])

if __name__ == "__main__":
    rospy.logout("Adding current state ang goal to the database...")
    add_knowledge_service = initialize_knowledge_client()
    rospy.sleep(2)
    recognize(add_knowledge_service)
    rospy.logout("State added. The system is ready to plan.")

    client_id= "KCL_rosplan"
    reference_name = "robot_explore"
    armor =  ArmorManager(client_id, reference_name)
    
    armor.add_ind_to_cls("wp4", "waypoint")
    armor.add_dataprop_to_ind("x","wp4","float","1.0")
    armor.add_dataprop_to_ind("y","wp4","float","2.0")
    armor.add_dataprop_to_ind("z","wp4","float","3.0")        
    res = armor.apply_changes()
    
    res = armor.query_dataprop_b2_ind("hasPosition", "wp4")

    print(res)