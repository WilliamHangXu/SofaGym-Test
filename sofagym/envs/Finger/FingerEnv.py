import math
import os
import sys

import numpy as np
from gym import spaces
from sofagym.AbstractEnv import AbstractEnv
from sofagym.rpc_server import start_scene

class FingerEnv(AbstractEnv):
    """Sub-class of AbstractEnv, dedicated to the cart pole scene.

    See the class AbstractEnv for arguments and methods.
    """
    path = path = os.path.dirname(os.path.abspath(__file__))
    metadata = {'render.modes': ['human', 'rgb_array']}
    DEFAULT_CONFIG = {"scene": "Finger",
                      "deterministic": True,
                      "source": [-100, 150, 300],
                      "target": [-100, 0, 0],
                      "goalList": [[-183.415, 29.3271, 19.347]],
                      "start_node": None,
                      "scale_factor": 2,
                      "dt": 0.001,
                      "timer_limit": 40,
                      "timeout": 25,
                      "display_size": (800, 400),
                      "render": 0,
                      "save_data": False,
                      "save_image": False,
                      "save_path": path + "/Results" + "/Finger",
                      "planning": False,
                      "discrete": False,
                      "start_from_history": None,
                      "python_version": sys.version,
                      "zFar": 4000,
                      "time_before_start": 100,
                      "seed": None
                      # "init_x": 0
                      # "max_move": 24
                      }
    
    def __init__(self, config = None):
        super().__init__(config)

        
        high = np.array(
            [
                [np.finfo(np.float32).max,np.finfo(np.float32).max,np.finfo(np.float32).max],
                [np.finfo(np.float32).max,np.finfo(np.float32).max,np.finfo(np.float32).max],
            ],
            dtype=np.float32,
        )
        low = np.array(
            [
                [-np.finfo(np.float32).max,-np.finfo(np.float32).max,-np.finfo(np.float32).max],
                [-np.finfo(np.float32).max,-np.finfo(np.float32).max,-np.finfo(np.float32).max],
            ],
            dtype=np.float32,
        )

        num_actions = 2
        self.action_space = spaces.Discrete(num_actions)
        self.num_actions = str(num_actions)

        self.observation_space = spaces.Box(high, low, dtype=np.float32)
    
    def formataction(self, action):
        return super()._formataction(action)
    
    def step(self, action):
        return super().step(action)
    
    def reset(self):
        """Reset simulation.

        Note:
        ----
            We launch a client to create the scene. The scene of the program is
            client_<scene>Env.py.

        """
        super().reset()

        self.config.update({'goalPos': self.goal})
        obs = start_scene(self.config, self.num_actions)

        return np.array(obs['observation'])
    

# ########## SIG 11 - SIGSEGV: segfault ##########
#   sofa::helper::BackTrace::sig(int)
#   PyErr_Fetch
#   on_exit+0
#   __libc_start_main
#   _start
# Exception in thread Thread-2 (deferredStart):
# Traceback (most recent call last):
#   File "/usr/lib/python3.10/threading.py", line 1016, in _bootstrap_inner
#     self.run()
#   File "/usr/lib/python3.10/threading.py", line 953, in run
#     self._target(*self._args, **self._kwargs)
#   File "/home/william/Documents/SofaGym/sofagym/rpc_server.py", line 527, in deferredStart
#     subprocess.run([sys.executable, path+"sofagym/rpc_client.py", sdict, str(nb_actions), str(port_rpc)],
#   File "/usr/lib/python3.10/subprocess.py", line 526, in run
#     raise CalledProcessError(retcode, process.args,
# subprocess.CalledProcessError: Command '['/usr/bin/python3.10', '/home/william/Documents/SofaGym/sofagym/rpc_client.py', "{'scene': 'Trunk', 'deterministic': True, 'source': [300, 0, 80], 'target': [0, 0, 80], 'goalList': [[40, 40, 100], [-10, 20, 80]], 'start_node': None, 'scale_factor': 5, 'timer_limit': 250, 'timeout': 50, 'display_size': (1600, 800), 'render': 1, 'save_data': False, 'save_image': False, 'save_path': '/home/william/Documents/SofaGym/sofagym/envs/Trunk/Results/Trunk', 'planning': False, 'discrete': True, 'seed': 0, 'start_from_history': None, 'python_version': 'python3', 'dt': 0.01, 'save_path_image': None, 'save_path_results': None, 'goal_node': 0, 'goalPos': [40, 40, 100]}", '16', '44201']'
