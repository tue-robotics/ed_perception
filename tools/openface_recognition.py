__author__ = 'amigo'

import os
import collections
import numpy as np
import cv2
from sklearn.svm import SVC
import openface
# In an image of a face, a ROI is set to the face only
# Then, the face is aligned, so that certain features of a face are always in the same place in an image,
#   which makes recognition easier.
# A face is represented as a point on a 128D unit hypersphere (https://cmusatyalab.github.io/openface/)
# If we do this for every image in our training set of only a few people,
#   each person should get a distinct (enough) representation and the points can be clustered.
#
# An unknown face should then be assigned to belong to one of the clusters.



fileDir = os.path.dirname(os.path.expanduser("~/git/openface/demos/"))
modelDir = os.path.join(fileDir, '..', 'models')
dlibModelDir = os.path.join(modelDir, 'dlib')
openfaceModelDir = os.path.join(modelDir, 'openface')

imgDim = 96

align = openface.AlignDlib(os.path.join(dlibModelDir, "shape_predictor_68_face_landmarks.dat"))
net = openface.TorchNeuralNet(os.path.join(openfaceModelDir, 'nn4.small2.v1.t7'), imgDim)

class FaceRecognizer(object):
    def __init__(self):
        self.traindata = []
        self.trainlabels = []

        self.classifier = SVC(C=1, kernel='linear', probability=True)

    def add_sample(self, name, imagePath):
        representation = self._represent(imagePath)

        self.traindata += [representation]
        self.trainlabels += [name]


    def cluster(self):
        self.classifier.fit(self.traindata, self.trainlabels)

    def _represent(self, imagePath):
        bgrImg = cv2.imread(imagePath)
        if bgrImg is None:
            raise Exception("Unable to load image: {}".format(imagePath))
        rgbImg = cv2.cvtColor(bgrImg, cv2.COLOR_BGR2RGB)


        bb = align.getLargestFaceBoundingBox(rgbImg)
        if bb is None:
            raise Exception("Unable to find a face: {}".format(imagePath))

        alignedFace = align.align(imgDim, rgbImg, bb, landmarkIndices=openface.AlignDlib.OUTER_EYES_AND_NOSE)
        if alignedFace is None:
            raise Exception("Unable to align image: {}".format(imagePath))

        rep = net.forward(alignedFace)

        return rep

    def classify(self, imagePath):
        representation = self._represent(imagePath)
        name = self.classifier.predict([representation])

        return name

def test():
    rec = FaceRecognizer()
    rec.add_sample('adams', "/home/amigo/git/openface/images/examples/adams.jpg")
    rec.add_sample('clapton', "/home/amigo/git/openface/images/examples/clapton-1.jpg")
    rec.add_sample('clapton', "/home/amigo/git/openface/images/examples/clapton-2.jpg")
    rec.add_sample('lennon', "/home/amigo/git/openface/images/examples/lennon-1.jpg")
    rec.add_sample('lennon', "/home/amigo/git/openface/images/examples/lennon-2.jpg")

    rec.cluster()

    name = rec.classify("/home/amigo/git/openface/images/examples/lennon-2.jpg")
    print name[0]
    assert name == "lennon"

if __name__ == "__main__":
    test()