'''In this file you need to implement remote procedure call (RPC) client

* The agent_server.py has to be implemented first (at least one function is implemented and exported)
* Please implement functions in ClientAgent first, which should request remote call directly
* The PostHandler can be implement in the last step, it provides non-blocking functions, e.g. agent.post.execute_keyframes
 * Hints: [threading](https://docs.python.org/2/library/threading.html) may be needed for monitoring if the task is done
'''

import weakref
import xmlrpc.client
from threading import Thread

from time import sleep
from numpy.matlib import identity
import numpy as np

import os
import sys
sys.path.append(os.path.join(os.path.abspath(os.path.dirname(__file__)), '..', 'joint_control'))

import keyframes

class PostHandler(object):
    '''the post hander wraps function to be excuted in paralle
    '''
    def __init__(self, obj):
        self.proxy = weakref.proxy(obj)

    def execute_keyframes(self, keyframes):
        '''non-blocking call of ClientAgent.execute_keyframes'''
        # YOUR CODE HERE
        thread = Thread(target=self.proxy.execute_keyframes, args=[keyframes])
        thread.start()

    def set_transform(self, effector_name, transform):
        '''non-blocking call of ClientAgent.set_transform'''
        # YOUR CODE HERE
        thread = Thread(target=self.proxy.set_transform, args=[effector_name, transform])
        thread.start()


class ClientAgent(object):
    '''ClientAgent request RPC service from remote server
    '''
    # YOUR CODE HERE
    def __init__(self):
        self.post = PostHandler(self)
        port = 5050
        self.proxy = xmlrpc.client.ServerProxy(f"http://localhost:{port}/")
    
    def get_angle(self, joint_name):
        '''get sensor value of given joint'''
        # YOUR CODE HERE
        return self.proxy.get_angle(joint_name)
    
    def set_angle(self, joint_name, angle):
        '''set target angle of joint for PID controller
        '''
        # YOUR CODE HERE
        return self.proxy.set_angle(joint_name, angle)

    def get_posture(self):
        '''return current posture of robot'''
        # YOUR CODE HERE
        return self.proxy.get_posture()

    def execute_keyframes(self, keyframes):
        '''excute keyframes, note this function is blocking call,
        e.g. return until keyframes are executed
        '''
        # YOUR CODE HERE
        return self.proxy.execute_keyframes(keyframes)

    def get_transform(self, name):
        '''get transform with given name
        '''
        # YOUR CODE HERE
        return np.asarray(self.proxy.get_transform(name))

    def set_transform(self, effector_name, transform):
        '''solve the inverse kinematics and control joints use the results
        '''
        # YOUR CODE HERE
        return self.proxy.set_transform(effector_name, transform.tolist())

if __name__ == '__main__':
    agent = ClientAgent()
    # TEST CODE HERE
    print(agent.get_angle("HeadYaw"))
    
    print(agent.set_angle("HeadYaw", 50))
    sleep(1)
    print(agent.get_angle("HeadYaw"))
    print(agent.get_posture())
    print(agent.set_angle("HeadYaw", 0))
    sleep(3)
    print(agent.post.execute_keyframes(keyframes.leftBackToStand()))
    sleep(1)
    while True:
        try:
            print(agent.get_posture())
        except:
            print("Loading Keyframes")
            continue
        break
    
    print(agent.get_transform("HeadYaw"))
    T = identity(4)
    T[-1, 0] = -0.5
    print(agent.post.set_transform("Head",T))
    
    sleep(0.1)
    while True:
        try:
            print(agent.get_transform("HeadYaw"))
        except:
            print("Loading set_transform")
            continue
        break
        
    print("Done")


