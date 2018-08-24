#include <string>
#include <ros/ros.h>
#include <vector>
#include "armor_msgs/ArmorDirective.h"
#include "armor_msgs/ArmorDirectiveRequest.h"
#include "armor_msgs/ArmorDirectiveList.h"
#include "rosplan_knowledge_msgs/KnowledgeItem.h"

armor_msgs::ArmorDirectiveRequest msgTemplate;

void setMsgDirective(armor_msgs::ArmorDirectiveRequest& msg, std::string command,
                                   std::string primarySpec, std::string secondarySpec) {
    msg.armor_request.command = command;
    msg.armor_request.primary_command_spec = primarySpec;
    msg.armor_request.secondary_command_spec = secondarySpec;
}
armor_msgs::ArmorDirectiveRequest newMessage(std::string command, std::string primarySpec,
                                                           std::string secondarySpec) {
    armor_msgs::ArmorDirectiveRequest newMsg = msgTemplate;
    setMsgDirective(newMsg, command, primarySpec, secondarySpec);
    return newMsg;
}

armor_msgs::ArmorDirectiveRequest newMessage(std::string command, std::string primarySpec,
                                                           std::string secondarySpec, std::vector<std::string> args) {
    armor_msgs::ArmorDirectiveRequest newMsg = msgTemplate;
    setMsgDirective(newMsg, command, primarySpec, secondarySpec);
    newMsg.armor_request.args = args;
    return newMsg;
}

bool applyChanges(ros::ServiceClient armorClient){
    // explicitly states all predicates and individuals are disjoint, needed for SWRL rules to work
    armor_msgs::ArmorDirectiveRequest req = newMessage("DISJOINT", "IND", "CLASS", {"Item"});
    armor_msgs::ArmorDirectiveResponse res;
    if(!armorClient.call(req, res) || !res.armor_response.success) return false;

    req = newMessage("APPLY", "", "", {});
    if(!armorClient.call(req, res) || !res.armor_response.success) return false;

    req = newMessage("REASON", "", "");
    return !(!armorClient.call(req, res) || !res.armor_response.success);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "rosplan_perception_manager");
  ros::NodeHandle nh;

  ROS_INFO("Waiting for ARMOR...");
  ros::service::waitForService("armor_interface_srv");
  ROS_INFO("Connected!");
  ros::ServiceClient armorClient = nh.serviceClient<armor_msgs::ArmorDirective>("armor_interface_srv", false);



  msgTemplate.armor_request.client_name = "KCL_rosplan";
  msgTemplate.armor_request.reference_name = "robot_explore";

  std::string entityName="wp4";
  armor_msgs::ArmorDirectiveResponse res;
  armor_msgs::ArmorDirectiveRequest req = newMessage("REMOVE", "IND", "", {entityName});
  armorClient.call(req, res) && applyChanges(armorClient);


  return 0;
}
