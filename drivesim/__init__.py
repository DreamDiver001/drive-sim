"""drivesim — a simulated variable-speed DC drive for practising test engineering."""
from .motor import DCMotor, MotorParams
from .controller import PIController
from .ramp import RampFunctionGenerator
from .drive import Drive, DriveConfig, DriveState, FaultCode
from .simulation import Simulation, Trace

__all__ = [
    "DCMotor", "MotorParams", "PIController", "RampFunctionGenerator",
    "Drive", "DriveConfig", "DriveState", "FaultCode", "Simulation", "Trace",
]
