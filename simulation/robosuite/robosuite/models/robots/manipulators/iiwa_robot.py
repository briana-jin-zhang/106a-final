import numpy as np
from robosuite.models.robots.manipulators.manipulator_model import ManipulatorModel
from robosuite.utils.mjcf_utils import xml_path_completion


class IIWA(ManipulatorModel):
    """
    IIWA is a bright and spunky robot created by KUKA

    Args:
        idn (int or str): Number or some other unique identification string for this robot instance
    """

    def __init__(self, idn=0):
        super().__init__(xml_path_completion("robots/iiwa/robot.xml"), idn=idn)

    @property
    def default_mount(self):
        return "RethinkMount"

    @property
    def default_gripper(self):
        return "Robotiq140Gripper"

    @property
    def default_controller_config(self):
        return "default_iiwa"

    @property
    def init_qpos(self):
    	return np.array([0.0, 0.0349066, 0.0, 0.523599, 0.0, -0.698132, 0.0])
        #return np.array([0.785398, -0.32724923, 0.000, 0.523599, 0.000, -0.698132, 0.000])

    @property
    def base_xpos_offset(self):
        return {
            "bins": (-0.5, -0.1, 0),
            "empty": (-0.6, 0, 0),
            "table": lambda table_length: (-0.16 - table_length/2, 0, 0)
        }

    @property
    def top_offset(self):
        return np.array((0, 0, 1.0))

    @property
    def _horizontal_radius(self):
        return 0.5

    @property
    def arm_type(self):
        return "single"
