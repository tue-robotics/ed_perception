#! /usr/bin/env python

import roslib;
import rospy
import actionlib
import sys

import ed_perception.msg

def learn_person(person_name, service_name):
    person_name

    print "Creating client"
    client = actionlib.SimpleActionClient(service_name, ed_perception.msg.FaceLearningAction)

    print "Waiting for server"
    client.wait_for_server()

    print "Creating goal with person_name: " + person_name
    goal = ed_perception.msg.FaceLearningGoal(person_name)

    print "Sending goal"
    client.send_goal(goal)

    print "Waiting for result"
    client.wait_for_result()

    print "Returning result"
    return client.get_result()

#---------------------------------------------------------------------------

if __name__ == '__main__':

    if len(sys.argv) > 1:
        person_name = sys.argv[1]
    else:
        print "Usage: learn_person.py <person_name>"
        exit(1)

    result = None
    service_name = "/amigo/ed/face_recognition/learn_face"

    try:
        print "Contacting learning service on " + service_name

        rospy.init_node('learn_person_py')

        result = learn_person(person_name = person_name, service_name = service_name)

        if result:
            if result.result_info == "Learning complete":
                print '\033[92m' + "Learning successful, result: '" + result.result_info + "'" + '\033[0m'
            else:
                print '\033[91m' + "Learning failed, result: '" + result.result_info + "'" + '\033[0m'
        else:
            print '\033[91m' + "Could not get a result" + '\033[0m'
        
    except rospy.ROSInterruptException:
        print '\033[91m' + "Program interrupted before completion" + '\033[0m'