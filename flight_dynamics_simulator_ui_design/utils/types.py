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


class PulseData(TypedDict):
    start_time: float
    duration: float
    angle_deg: float