# controller.py

from enum import Enum


# Define the controller types as an enumeration
class ControllerType(Enum):
    NONE = 0            # No control action, no feedback
    BANG = 1            # Bang-bang control, unidirectional
    BANG2 = 2           # Bang-bang control, bipolar
    BANGH = 3           # Bang-bang control with hysteresis
    P = 4               # Proportional control
    PID = 5             # PID (Proportional-Integral-Derivative) control

# Active controller
active_controller = ControllerType.P

# Constants for the controller
h = 0.050  # Sampling interval
Kp = 100.0  # Proportional constant
Ti = float('inf')  # Integration time (set to infinity to disable I component)
Td = 0 * h  # Differential time
max_u = 10.0  # Saturation value for control signal
deltah = 0.05  # Hysteresis for bang-bang controller
bangvalue = 0.5  # Value for bang-bang control

# Controller function
def controller(type, r, y):
    u = 0.0  # Control signal

    # Auxiliary constants for the PID controller
    K0 = Kp * (1 + h / Ti + Td / h)
    K1 = -Kp * (1 + 2 * Td / h)
    K2 = Kp * Td / h

    # Memory for error
    e_m1 = 0.0
    e_m2 = 0.0

    # Memory for the control signal
    u_m1 = 0.0

    # Compute error signal
    e = r - y

    # Implement control action depending on the type of control
    if type == ControllerType.NONE:
        # No feedback action, output is the reference input
        u = r
    elif type == ControllerType.BANG:
        # Bang-bang control, unidirectional
        u = bangvalue if e > 0 else 0.0
    elif type == ControllerType.BANG2:
        # Bang-bang control, bipolar
        if e > 0:
            u = bangvalue
        elif e < 0:
            u = -bangvalue
        else:
            u = 0.0
    elif type == ControllerType.BANGH:
        # Bang-bang control with hysteresis
        if e > 0.5 * deltah:
            u = bangvalue
        elif e < -0.5 * deltah:
            u = -bangvalue
        else:
            u = u_m1  # Keep the previous value
    elif type == ControllerType.P:
        # Proportional control
        u = Kp * e
    elif type == ControllerType.PID:
        # Compute control signal
        u = u_m1 + K0 * e + K1 * e_m1 + K2 * e_m2
        
        # Store values for the next iterations
        e_m2 = e_m1
        e_m1 = e
        u_m1 = u

        # Clip the control signal to avoid saturation
        u = max(min(u, max_u), -max_u)

    return u
