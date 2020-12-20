'''In this exercise you need to implement forward kinematics for NAO robot

* Tasks:
    1. complete the kinematics chain definition (self.chains in class ForwardKinematicsAgent)
       The documentation from Aldebaran is here:
       http://doc.aldebaran.com/2-1/family/robots/bodyparts.html#effector-chain
    2. implement the calculation of local transformation for one joint in function
       ForwardKinematicsAgent.local_trans. The necessary documentation are:
       http://doc.aldebaran.com/2-1/family/nao_h21/joints_h21.html
       http://doc.aldebaran.com/2-1/family/nao_h21/links_h21.html
    3. complete function ForwardKinematicsAgent.forward_kinematics, save the transforms of all body parts in torso
       coordinate into self.transforms of class ForwardKinematicsAgent

* Hints:
    the local_trans has to consider different joint axes and link parameters for different joints
'''

# add PYTHONPATH
import os
import sys
sys.path.append(os.path.join(os.path.abspath(os.path.dirname(__file__)), '..', 'joint_control'))

from numpy.matlib import matrix, identity
import numpy as np

from angle_interpolation import AngleInterpolationAgent

def translation_matrix(x, y, z):
    T = identity(4)
    T[0, [3]] = x
    T[1, [3]] = y
    T[2, [3]] = z
    return T

def rotation_matrix(axis, angle):
    matrix_dict = {
        'x': matrix([[1, 0, 0, 0], [0, np.cos(angle), -np.sin(angle), 0], [0, np.sin(angle), np.cos(angle), 0], [0, 0, 0, 1]]),
        'y': matrix([[np.cos(angle), 0, np.sin(angle), 0], [0, 1, 0, 0], [-np.sin(angle), 0, np.cos(angle), 0], [0, 0, 0, 1]]),
        'z': matrix([[np.cos(angle), -np.sin(angle), 0, 0], [np.sin(angle), np.cos(angle), 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])
    }
    return matrix_dict.get(axis)

class ForwardKinematicsAgent(AngleInterpolationAgent):
    def __init__(self, simspark_ip='localhost',
                 simspark_port=3100,
                 teamname='DAInamite',
                 player_id=0,
                 sync_mode=True):
        super(ForwardKinematicsAgent, self).__init__(simspark_ip, simspark_port, teamname, player_id, sync_mode)
        self.transforms = {n: identity(4) for n in self.joint_names}

        # chains defines the name of chain and joints of the chain
        self.chains = {'Head': ['HeadYaw', 'HeadPitch'],
                       'LArm': ['LShoulderPitch', 'LShoulderRoll', 'LElbowYaw', 'LElbowRoll'], #, 'LWristYaw', 'LHand'
                       'LLeg': ['LHipYawPitch', 'LHipRoll', 'LHipPitch', 'LKneePitch', 'LAnklePitch', 'LAnkleRoll'],
                       'RLeg': ['RHipYawPitch', 'RHipRoll', 'RHipPitch', 'RKneePitch', 'RAnklePitch', 'RAnkleRoll'],
                       'RArm': ['RShoulderPitch', 'RShoulderRoll', 'RElbowYaw', 'RElbowRoll'], #, 'RWristYaw', 'RHand'
                       }

    def think(self, perception):
        self.forward_kinematics(perception.joint)
        return super(ForwardKinematicsAgent, self).think(perception)

    def local_trans(self, joint_name, joint_angle):
        '''calculate local transformation of one joint

        :param str joint_name: the name of joint
        :param float joint_angle: the angle of joint in radians
        :return: transformation
        :rtype: 4x4 matrix
        '''
        T = identity(4)
        local_transforms_dict = {
            #'joint': np.matmul(translation_matrix(x, y, z), rotation_matrix(axis, joint_angle)) #for axis in {'x', 'y', 'z'}
            'HeadYaw': np.matmul(translation_matrix(0.0, 0.0, 126.5), rotation_matrix('z', joint_angle)),
            'HeadPitch': np.matmul(translation_matrix(0.0, 0.0, 0.0), rotation_matrix('y', joint_angle)),
            # left arm
            'LShoulderPitch': np.matmul(translation_matrix(0.0, 98.0, 100.0), rotation_matrix('y', joint_angle)),
            'LShoulderRoll': np.matmul(translation_matrix(0.0, 0.0, 0.0), rotation_matrix('z', joint_angle)),
            'LElbowYaw': np.matmul(translation_matrix(105.0, 15.0, 0.0), rotation_matrix('x', joint_angle)),
            'LElbowRoll': np.matmul(translation_matrix(0.0, 0.0, 0.0), rotation_matrix('z', joint_angle)),
            # right arm
            'RShoulderPitch': np.matmul(translation_matrix(0.0, -98.0, 100.0), rotation_matrix('y', joint_angle)),
            'RShoulderRoll': np.matmul(translation_matrix(0.0, 0.0, 0.0), rotation_matrix('z', joint_angle)),
            'RElbowYaw': np.matmul(translation_matrix(105.0, -15.0, 0.0), rotation_matrix('x', joint_angle)),
            'RElbowRoll': np.matmul(translation_matrix(0.0, 0.0, 0.0), rotation_matrix('z', joint_angle)),
            # left leg
            'LHipYawPitch': np.matmul(translation_matrix(0.0, 50.0, -85.0), np.matmul(rotation_matrix('z', joint_angle), rotation_matrix('y', joint_angle))),
            'LHipRoll': np.matmul(translation_matrix(0.0, 0.0, 0.0), rotation_matrix('x', joint_angle)),
            'LHipPitch': np.matmul(translation_matrix(0.0, 0.0, 0.0), rotation_matrix('y', joint_angle)),
            'LKneePitch': np.matmul(translation_matrix(0.0, 0.0, -100.0), rotation_matrix('y', joint_angle)),
            'LAnklePitch': np.matmul(translation_matrix(0.0, 0.0, -102.0), rotation_matrix('y', joint_angle)),
            'LAnkleRoll': np.matmul(translation_matrix(0.0, 0.0, 0.0), rotation_matrix('x', joint_angle)),
            # right leg
            'RHipYawPitch': np.matmul(translation_matrix(0.0, -50.0, -85.0), np.matmul(rotation_matrix('z', joint_angle), rotation_matrix('y', joint_angle))),
            'RHipRoll': np.matmul(translation_matrix(0.0, 0.0, 0.0), rotation_matrix('x', joint_angle)),
            'RHipPitch': np.matmul(translation_matrix(0.0, 0.0, 0.0), rotation_matrix('y', joint_angle)),
            'RKneePitch': np.matmul(translation_matrix(0.0, 0.0, -100.0), rotation_matrix('y', joint_angle)),
            'RAnklePitch': np.matmul(translation_matrix(0.0, 0.0, -102.0), rotation_matrix('y', joint_angle)),
            'RAnkleRoll': np.matmul(translation_matrix(0.0, 0.0, 0.0), rotation_matrix('x', joint_angle)),
        }

        return local_transforms_dict.get(joint_name)

    def forward_kinematics(self, joints):
        '''forward kinematics

        :param joints: {joint_name: joint_angle}
        '''
        for chain_joints in self.chains.values():
            T = identity(4)
            for joint in chain_joints:
                angle = joints[joint]
                Tl = self.local_trans(joint, angle)
                T = np.matmul(Tl, T)

                self.transforms[joint] = T

if __name__ == '__main__':
    agent = ForwardKinematicsAgent()
    agent.run()
