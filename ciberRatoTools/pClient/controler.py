# controller.py

from controller_constants import ControllerType


class Controller:
    def __init__(self, sampling_interval=0.050, Kp=100, Ti=float('inf'), Td=0):
        self.h = sampling_interval
        self.Kp = Kp
        self.Ti = Ti
        self.Td = Td
        self.max_u = 10
        self.deltah = 0.05
        self.bangvalue = 0.5
        
        # Memory for errors and control signals
        self.e_m1 = 0
        self.e_m2 = 0
        self.u_m1 = 0

    def controller(self, type, r, y):
        """ Compute control signal based on the controller type. """
        u = 0  # Control signal

        K0 = self.Kp * (1 + self.h / self.Ti + self.Td / self.h)
        K1 = -self.Kp * (1 + 2 * self.Td / self.h)
        K2 = self.Kp * self.Td / self.h

        # Compute error signal
        e = r - y

        # Implement control action depending on the type of control
        if type == ControllerType.NONE:
            u = r  # No feedback action
        elif type == ControllerType.BANG:
            u = self.bangvalue if e > 0 else 0
        elif type == ControllerType.BANG2:
            if e > 0:
                u = self.bangvalue
            elif e < 0:
                u = -self.bangvalue
            else:
                u = 0
        elif type == ControllerType.BANGH:
            if e > 0.5 * self.deltah:
                u = self.bangvalue
            elif e < -0.5 * self.deltah:
                u = -self.bangvalue
            else:
                u = self.u_m1  # Keep the last value
            self.u_m1 = u
        elif type == ControllerType.P:
            u = self.Kp * e
        elif type == ControllerType.PID:
            # Compute control signal
            u = self.u_m1 + K0 * e + K1 * self.e_m1 + K2 * self.e_m2

            # Store values for next iterations
            self.e_m2 = self.e_m1
            self.e_m1 = e
            self.u_m1 = u

            # Clip the control signal to avoid saturation
            u = max(min(u, self.max_u), -self.max_u)

        return u
