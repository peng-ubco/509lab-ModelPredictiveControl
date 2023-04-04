import numpy as np
from simulator.sim_1d import run_sim

# Simulator options.
sim_opt = {}
sim_opt['FULL_RECALCULATE'] = False
sim_opt['FIG_SIZE'] = [8, 8]


class MPC:
    def __init__(self):
        self.horizon = 20  # The length of the prediction
        self.dt = 0.1

        # Set point that the controller will achieve.
        self.reference = [50, 0, 0]

    def system_model(self, prev_state, dt, pedal, steering=0):
        # Assume we know the state of the car,
        # and state_vector=[pos_x, pos_y, car_angle, velocity],
        # Two control inputs: pedal: pedal position, and steering: steering angle

        # Get the state of the car
        pos_x = prev_state[0]
        v = prev_state[3]  # m/s

        '''your code starts here'''
        '''***********************************************'''

        '''***********************************************'''
        # Return the predicted new state
        return [pos_x, 0, 0, v]


    def cost_func(self, u, state, ref):
    # def cost_func(self, u, *args):
        # u is the control inputs
        # state = args[0]  # current state
        # ref = args[1]  # reference or set point
        cost = 0.0
        
        '''your code starts here'''
        '''***********************************************'''

        '''***********************************************'''
        return cost

run_sim(sim_opt, MPC)

