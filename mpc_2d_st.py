import numpy as np
from simulator.sim_2d import sim_run

# Simulator options.
sim_opt = {}
sim_opt['OBSTACLES'] = False
sim_opt['FIG_SIZE'] = [8, 8]


class MPC:
    def __init__(self):
        self.horizon = 10
        self.dt = 0.1

        # Reference or set point 
        self.reference1 = [10, 10, 0]
        self.reference2 = [10, 2, 3*3.14/2]

    def motion_model(self, state, dt, pedal, steering):
        # Assume we know the state of the car,
        # and state_vector=[pos_x, pos_y, car_angle, velocity]
        # Two control inputs: pedal: pedal position, and steering: steering angle

        # Get the state of the car
        pos_x = state[0]
        pos_y = state[1]
        psi = state[2]
        v = state[3]  # m/s

        '''your code starts here'''
        '''***********************************************'''

        '''***********************************************'''

        # Return the predicted new state
        return [pos_x, pos_y, psi, v]

    def cost_func(self, u, *args):
        state = args[0]
        ref = args[1]
        cost = 0.0

        '''your code starts here'''
        '''***********************************************'''

        '''***********************************************'''

        return cost

sim_run(sim_opt, MPC)


