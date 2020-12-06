'''In this exercise you need to use the learned classifier to recognize current posture of robot

* Tasks:
    1. load learned classifier in `PostureRecognitionAgent.__init__`
    2. recognize current posture in `PostureRecognitionAgent.recognize_posture`

* Hints:
    Let the robot execute different keyframes, and recognize these postures.

'''


from os import listdir, path
from angle_interpolation import AngleInterpolationAgent
from keyframes import *
from sklearn import svm, metrics
import numpy as np

import pickle


class PostureRecognitionAgent(AngleInterpolationAgent):
    def __init__(self, simspark_ip='localhost',
                 simspark_port=3100,
                 teamname='DAInamite',
                 player_id=0,
                 sync_mode=True):
        super(PostureRecognitionAgent, self).__init__(simspark_ip, simspark_port, teamname, player_id, sync_mode)
        self.posture = 'unknown'
        self.posture_classifier = pickle.load(open("robot_pose.pkl", 'rb'))
        self.posture_dict = {
            0: 'Sit',
            1: 'Back',
            2: 'Right',
            3: 'Crouch',
            4: 'Left',
            5: 'Frog',
            6: 'Stand',
            7: 'Knee',
            8: 'HeadBack',
            9: 'StandInit',
            10: 'Belly',
        }

    def think(self, perception):
        self.posture = self.recognize_posture(perception)
        return super(PostureRecognitionAgent, self).think(perception)

    def recognize_posture(self, perception):
        posture = 'unknown'
        posture_data = [[]]
        features = ['LHipYawPitch', 'LHipRoll', 'LHipPitch', 'LKneePitch', 'RHipYawPitch', 'RHipRoll', 'RHipPitch', 'RKneePitch']
        for feature in features:
            posture_data[0].append(self.perception.joint[feature])
        posture_data[0].append(self.perception.imu[0])
        posture_data[0].append(self.perception.imu[1])

        posture = self.posture_classifier.predict(posture_data)
        posture = self.posture_dict[posture[0]]

        return posture

if __name__ == '__main__':
    agent = PostureRecognitionAgent()
    agent.keyframes = hello()  # CHANGE DIFFERENT KEYFRAMES
    agent.run()
