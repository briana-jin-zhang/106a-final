from collections import OrderedDict

class RobotModel:
    """
    Base class of robot model from urdf file
    When initialized, loads links and joints of robot.
    """
    def __init__(self):
        self._links = OrderedDict()
        self._joints = OrderedDict()

    def find_frame(self):
        """
        Find robot's frame
        """
        raise NotImplementedError

    def find_link(self):
        """
        Find robot's link
        """
        raise NotImplementedError

    def find_joint(self):
        """
        Find robot's joint
        """
        raise NotImplementedError

    @property
    def links(self):
        """
        Returns:
            OrderedDict: all links
        """
        return self._links

    @property
    def joints(self):
        """
        Returns:
            OrderedDict: all joints
        """
        return self._joints

    @property
    def dof(self):
        """
        Robot's dof
        """
        raise NotImplementedError

    @property
    def num_links(self):
        """
        Number of links
        """
        raise NotImplementedError

    @property
    def num_joints(self):
        """
        Number of joints
        """
        raise NotImplementedError

    @property
    def num_fixed_joints(self):
        """
        Number of fixed joints
        """
        raise NotImplementedError

    @property
    def num_actuated_joints(self):
        """
        Number of actuated(revolute or prismatic) joints
        """
        raise NotImplementedError

    @property
    def num_revolute_joints(self):
        """
        Number of revolute joints
        """
        raise NotImplementedError