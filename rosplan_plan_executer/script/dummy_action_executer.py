

import sys
import rospy

from std_msgs.msg import *
from diagnostic_msgs.msg import KeyValue
from rosplan_dispatch_msgs.msg import *

import rosplan_knowledge_msgs.srv
import rosplan_knowledge_msgs.msg
import diagnostic_msgs.msg

def initialize_knowledge_client():
    rospy.wait_for_service('/kcl_rosplan/update_knowledge_base')
    try:
        service = rospy.ServiceProxy('/kcl_rosplan/update_knowledge_base',
                                     rosplan_knowledge_msgs.srv.KnowledgeUpdateService)
        return service
    except rospy.ServiceException, e:
        print "Service call failed: %s" % e

if __name__ == "__main__":
    rospy.logout("Adding current state ang goal to the database...")
    add_knowledge_service = initialize_knowledge_client()
    rospy.sleep(2)