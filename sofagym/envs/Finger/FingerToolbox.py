import numpy as np

import Sofa
import Sofa.Core
import Sofa.Simulation
import SofaRuntime
from splib3.animation.animate import Animation

SofaRuntime.importPlugin("Sofa.Component")

def action_to_command(action):
    if action == 0:
        displacement = 0.5
    elif action == 1:
        displacement = -0.5
    elif action == 2:
        displacement = 0
    else:
        raise NotImplementedError("Action must be in range 0 - 2")
    return displacement

def displace(cable, displacement,factor):
    total_displacement = cable.CableConstraint.value[0]
    total_displacement += displacement
    if total_displacement > 62:
        total_displacement = 62
    if total_displacement < 0:
        total_displacement = 0
    cable.CableConstraint.value = [total_displacement]
    print(total_displacement)

def startCmd_Finger(rootNode, cable, displacement, duration):
    """Initialize the command.

    Parameters:
    ----------
        rootNode: <Sofa.Core>
            The scene.
        cable: <MechanicalObject>
            The mechanical object of the cable to move.
        displacement: float
            The elements of the commande.
        duration: float
            Duration of the animation.

    Returns:
    -------
        None.
    """

    # Definition of the elements of the animation
    # print("startcmd_finger")
    # def executeAnimation(cable, displacement, factor):
    #     displace(cable, displacement)

    # Add animation in the scene
    # breakpoint()
    rootNode.AnimationManager.addAnimation(
        Animation(
            onUpdate=displace,
            params={"cable": cable,
                    "displacement": displacement},
            duration=duration, mode="once"))
    

def startCmd(root, actions, duration):
    """Initialize the command from root and action.

    Parameters:
    ----------
        rootNode: <Sofa.Core>
            The scene.
        action: int
            The action.
        duration: float
            Duration of the animation.

    Returns:
    ------
        None.

    """
    incr = action_to_command(actions)
    startCmd_Finger(root, root.Simulation.Finger.ElasticMaterialObject.PullingCable, incr, duration)

class rewardShaper(Sofa.Core.Controller):
    def __init__(self, *args, **kwargs):
        """Initialization of all arguments.

        Parameters:
        ----------
            kwargs: Dictionary
                Initialization of the arguments.

        Returns:
        -------
            None.

        """
        Sofa.Core.Controller.__init__(self, *args, **kwargs)

        self.rootNode = None
        if kwargs["rootNode"]:
            self.rootNode = kwargs["rootNode"]
        self.goal_pos = None
        if kwargs["goalPos"]:
            self.goal_pos = kwargs["goalPos"]

    def getReward(self):
        self_pos = self.rootNode.Simulation.Finger.ElasticMaterialObject.dofs.position[149]
        current_dist = np.linalg.norm(np.array(self_pos)-np.array(self.goal_pos))
        reward = max((self.prev_dist - current_dist)/self.prev_dist, 0.0)
        self.prev_dist = current_dist
        return reward, current_dist
    
    def update(self):
        """Update function.

        This function is used as an initialization function.

        Parameters:
        ----------
            None.

        Arguments:
        ---------
            None.

        """
        self_pos = self.rootNode.Simulation.Finger.ElasticMaterialObject.dofs.position[149]
        self.init_dist = np.linalg.norm(np.array(self_pos)-np.array(self.goal_pos))
        self.prev_dist = self.init_dist

def getReward(rootNode):
    reward, current_dist = rootNode.Reward.getReward()
    done = (current_dist < 1.0).item()

    return done, reward

def getState(rootNode):
    # print("position")
    # print(rootNode.Simulation.Finger.ElasticMaterialObject.dofs.position[149])
    # print("velocity")
    # print(rootNode.Simulation.Finger.ElasticMaterialObject.dofs.velocity[149])
    state = [rootNode.Simulation.Finger.ElasticMaterialObject.dofs.position[149].tolist(), rootNode.Simulation.Finger.ElasticMaterialObject.dofs.velocity[149].tolist()]
    return state

def getPos(root):
    print("boom1")
    return root.Simulation.Finger.ElasticMaterialObject.dofs.position.value.tolist()

def setPos(root, pos):
    print("boom2")
    root.Simulation.Finger.ElasticMaterialObject.dofs.position.value = np.array(pos)

class goalSetter(Sofa.Core.Controller):
    """Compute the goal.

    Methods:
    -------
        __init__: Initialization of all arguments.
        update: Initialize the value of cost.

    Arguments:
    ---------
        goalMO: <MechanicalObject>
            The mechanical object of the goal.
        goalPos: coordinates
            The coordinates of the goal.

    """

    def __init__(self, *args, **kwargs):
        """Initialization of all arguments.

        Parameters:
        ----------
            kwargs: Dictionary
                Initialization of the arguments.

        Returns:
        -------
            None.

        """
        Sofa.Core.Controller.__init__(self, *args, **kwargs)

        self.goalMO = None
        if kwargs["goalMO"]:
            self.goalMO = kwargs["goalMO"]
        self.goalPos = None
        if kwargs["goalPos"]:
            self.goalPos = kwargs["goalPos"]

    def update(self):
        """Set the position of the goal.

        This function is used as an initialization function.

        Parameters:
        ----------
            None.

        Arguments:
        ---------
            None.

        """
        with self.goalMO.position.writeable() as position:
            position += self.goalPos

    def set_mo_pos(self, goal):
        """Modify the goal.

        Not used here.
        """
        pass

