#!/usr/bin/env python

# ROS
import rospy

# TU/e Robotics
from robocup_knowledge.knowledge_loader import load_knowledge

# Main function
if __name__ == "__main__":

    # Launch the ROS node
    rospy.init_node("load_object_types")

    # Load the knowledge
    knowledge = load_knowledge("common")

    # Get all objects from the knowledge
    objects = knowledge.get_objects()

    # Dump all objects on the parameter server
    rospy.set_param("/object_types", objects)
