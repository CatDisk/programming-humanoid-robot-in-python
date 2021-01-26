'''In this file you need to implement remote procedure call (RPC) client

* The agent_server.py has to be implemented first (at least one function is implemented and exported)
* Please implement functions in ClientAgent first, which should request remote call directly
* The PostHandler can be implement in the last step, it provides non-blocking functions, e.g. agent.post.execute_keyframes
 * Hints: [threading](https://docs.python.org/2/library/threading.html) may be needed for monitoring if the task is done
'''

import weakref

#imports for RCP client
import requests
import json

class PostHandler(object):
    '''the post hander wraps function to be excuted in paralle
    '''
    def __init__(self, obj):
        self.proxy = weakref.proxy(obj)

    def execute_keyframes(self, keyframes):
        '''non-blocking call of ClientAgent.execute_keyframes'''
        # YOUR CODE HERE

    def set_transform(self, effector_name, transform):
        '''non-blocking call of ClientAgent.set_transform'''
        # YOUR CODE HERE
        '''solve the inverse kinematics and control joints use the results
        '''
        # YOUR CODE HERE
        temp_payload = self.proxy.payload.copy()
        temp_payload["method"] = "set_transform"
        temp_payload["params"] = [effector_name, transform]
        self.proxy.__post_request(temp_payload)


class ClientAgent(object):
    '''ClientAgent request RPC service from remote server
    '''
    # YOUR CODE HERE
    def __init__(self):
        self.post = PostHandler(self)
        self.url = "http://localhost:4000/jsonrpc"
        self.payload = {
            "method": "",
            "params": [],
            "jsonrpc": "2.0",
            "id": 0,
        }
    
    def get_angle(self, joint_name):
        '''get sensor value of given joint'''
        # YOUR CODE HERE
        temp_payload = self.payload.copy()
        temp_payload["method"] = "get_angle"
        temp_payload["params"] = [joint_name]
        self.post_request(temp_payload)
    
    def set_angle(self, joint_name, angle):
        '''set target angle of joint for PID controller
        '''
        # YOUR CODE HERE
        temp_payload = self.payload.copy()
        temp_payload["method"] = "set_angle"
        temp_payload["params"] = [joint_name, angle]
        self.post_request(temp_payload)

    def get_posture(self):
        '''return current posture of robot'''
        # YOUR CODE HERE
        temp_payload = self.payload.copy()
        temp_payload["method"] = "get_posture"
        self.post_request(temp_payload)

    def execute_keyframes(self, keyframes):
        '''excute keyframes, note this function is blocking call,
        e.g. return until keyframes are executed
        '''
        # YOUR CODE HERE
        temp_payload = self.payload.copy()
        temp_payload["method"] = "execute_keyframes"
        temp_payload["params"] = [keyframes]
        self.post_request(temp_payload)

    def get_transform(self, name):
        '''get transform with given name
        '''
        # YOUR CODE HERE
        temp_payload = self.payload.copy()
        temp_payload["method"] = "get_transform"
        temp_payload["params"] = [name]
        self.post_request(temp_payload)

    def set_transform(self, effector_name, transform):
        '''solve the inverse kinematics and control joints use the results
        '''
        # YOUR CODE HERE
        temp_payload = self.payload.copy()
        temp_payload["method"] = "set_transform"
        temp_payload["params"] = [effector_name, transform]
        self.post_request(temp_payload)
    
    def post_request(self, payload):
        response = requests.post(self.url, json=payload).json()
        try:
            print(response["result"])
        except KeyError:
            try:
                error = response["error"]
                print(error["message"] + ": " + payload["method"])
            except:
                print("Something else went wrong")

if __name__ == '__main__':
    agent = ClientAgent()
    conc_agent = PostHandler(agent)
    conc_agent.set_transform("Name", "1234")
    agent.get_posture()


