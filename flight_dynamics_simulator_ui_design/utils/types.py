from typing import TypedDict, Union

UAVParameterValue = Union[str, float]


class UAVParameter(TypedDict):
    company: str
    country: str
    mass: float
    S: float
    c: float
    b: float
    Iyy: float
    Mach: float
    CL_0: float
    CL_alpha: float
    CL_q: float
    CL_deltae: float
    CL_u: float
    CD_0: float
    CD_alpha: float
    CD_q: float
    CD_deltae: float
    CD_u: float
    Cm_0: float
    Cm_alpha: float
    Cm_q: float
    Cm_deltae: float
    Cm_u: float
    # 6DOF additions
    ixx: float
    izz: float
    ixz: float
    CYb: float
    CYp: float
    CYr: float
    CYda: float
    CYdr: float
    Clb: float
    Clp: float
    Clr: float
    Clda: float
    Cldr: float
    Cnb: float
    Cnp: float
    Cnr: float
    Cnda: float
    Cndr: float


class PulseData(TypedDict):
    start_time: float
    duration: float
    angle_deg: float  # Elevator (4DOF/6DOF pitch)
    # 6DOF additions (optional for 4DOF, used in 6DOF)
    roll_deg: float  # Aileron/roll input
    yaw_deg: float   # Rudder/yaw input
    throttle: float  # Throttle input (0-1 or percent)