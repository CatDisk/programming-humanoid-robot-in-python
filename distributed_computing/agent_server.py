'''In this file you need to implement remote procedure call (RPC) server

* There are different RPC libraries for python, such as xmlrpclib, json-rpc. You are free to choose.
* The following functions have to be implemented and exported:
 * get_angle
 * set_angle
 * get_posture
 * execute_keyframes
 * get_transform
 * set_transform
* You can test RPC server with ipython before implementing agent_client.py
'''

# add PYTHONPATH
import os
import sys
sys.path.append(os.path.join(os.path.abspath(os.path.dirname(__file__)), '..', 'kinematics'))
import time

# imports for RPC server
from jsonrpc import dispatcher
from jsonrpc.manager import JSONRPCResponseManager
from werkzeug.wrappers import Request, Response
from werkzeug.serving import run_simple



from inverse_kinematics import InverseKinematicsAgent
from forward_kinematics import ForwardKinematicsAgent
from recognize_posture import PostureRecognitionAgent
from angle_interpolation import AngleInterpolationAgent

class ServerAgent(InverseKinematicsAgent):
    '''ServerAgent provides RPC service
    '''
    # YOUR CODE HERE
    d = dispatcher.Dispatcher()
     
    def get_angle(self, joint_name):
        '''get sensor value of given joint'''
        return self.perception.joint[joint_name]
     
    def set_angle(self, joint_name, angle):
        '''set target angle of joint for PID controller
        '''
        # YOUR CODE HERE
        return "set_angle: " + joint_name + str(angle)
     
    def get_posture(self):
        '''return current posture of robot'''
        # YOUR CODE HERE
        return "get posture"
     
    def execute_keyframes(self, keyframes):
        '''excute keyframes, note this function is blocking call,
        e.g. return until keyframes are executed
        '''
        # YOUR CODE HERE
        return "exec: " + keyframes
     
    def get_transform(self, name):
        '''get transform with given name
        '''
        # YOUR CODE HERE
        return "get transform: " + name
     
    def set_transform(self, effector_name, transform):
        '''solve the inverse kinematics and control joints use the results
        '''
        # YOUR CODE HERE
        print("sleepo beepo")
        time.sleep(3.0)
        return "set transform: " + effector_name + transform


if __name__ == '__main__':
    agent = ServerAgent()
    agent.d["get_angle"] = agent.get_angle
    agent.d["set_angle"] = agent.set_angle
    agent.d["get_posture"] = agent.get_posture
    agent.d["execute_keyframes"] = agent.execute_keyframes
    agent.d["get_transform"] = agent.get_transform
    agent.d["set_transform"] = agent.set_transform
    @Request.application
    def application(request):
        print(request.data)
        response = JSONRPCResponseManager.handle(request.data, agent.d)
        return Response(response.json, mimetype='application/json')
    run_simple('localhost', 4000, application, threaded=True)
    agent.run()

