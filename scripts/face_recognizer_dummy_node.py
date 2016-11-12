#!/usr/bin/env python

import rospy

from image_recognition_msgs.srv import Recognize, Annotate
from image_recognition_msgs.msg import Recognition, CategoryProbability, CategoricalDistribution
from sensor_msgs.msg import RegionOfInterest
from std_srvs.srv import Empty


class FaceRecognizerDummyROS:
    def __init__(self):
        self._labels = []
        self._annotate_srv = rospy.Service('annotate', Annotate, self._annotate_srv)
        self._recognize_srv = rospy.Service('recognize', Recognize, self._recognize_srv)
        self._clear_srv = rospy.Service('clear', Empty, self._clear_srv)
        rospy.loginfo("Started FaceRecognizerDummyROS")

    def _annotate_srv(self, req):
        for annotation in req.annotations:
            rospy.loginfo("Adding face for %s" % annotation.label)
            self._labels.append(annotation.label)

        return {}

    def _clear_srv(self, req):
        rospy.loginfo("Clearing all faces")
        self._labels = []
        return {}

    def _recognize_srv(self, req):
        # Fill recognitions
        recognitions = []

        recognitions.append(Recognition(
            categorical_distribution=CategoricalDistribution(
                    unknown_probability=0.0,
                probabilities=[CategoryProbability(label=label, probability=0.5) for label in self._labels]
            ),
            roi=RegionOfInterest(
                x_offset=req.image.width/2,
                y_offset=req.image.height/2,
                width=req.image.width/4,
                height=req.image.height/4
            )
        ))

        # Service response
        return {"recognitions": recognitions}

if __name__ == '__main__':

    rospy.init_node("face_recognition_dummy")

    openface_ros = FaceRecognizerDummyROS()
    rospy.spin()
