"""
Class to determine wind velocity at any given moment,
calculates a steady wind speed and uses a stochastic
process to represent wind gusts. (Follows section 4.4 in uav book)
"""
from tools.transfer_function import TransferFunction
import parameters.wind_parameters as WIND_PARAMS
import parameters.aerosonde_parameters as MAV


import numpy as np


class WindSimulation:
    def __init__(self, Ts, gust_flag = True, steady_state = np.array([[0., 0., 0.]]).T):
        # steady state wind defined in the inertial frame
        self._steady_state = steady_state
        ##### TODO #####
        sigma_u = WIND_PARAMS.sigma_u
        sigma_v = WIND_PARAMS.sigma_v
        sigma_w = WIND_PARAMS.sigma_w
        L_u = WIND_PARAMS.L_u
        L_v = WIND_PARAMS.L_v
        L_w = WIND_PARAMS.L_w
        Va = MAV.Va0  # constant nominal airspeed
        #   Dryden gust model parameters (pg 56 UAV book)
        u_a0 = sigma_u * np.sqrt(2. * Va / np.pi / L_u)
        u_b1 = 1
        u_b0 = Va / L_u

        v_a1 = sigma_v * np.sqrt(3. * Va / np.pi / L_v)
        v_a0 = v_a1 * Va / np.sqrt(3) / L_v
        v_b2 = 1
        v_b1 = 2 * Va / L_v
        v_b0 = (v_b1 / 2)**2

        w_a1 = sigma_w * np.sqrt(3. * Va / np.pi / L_w)
        w_a0 = w_a1 * Va / np.sqrt(3) / L_w
        w_b2 = 1
        w_b1 = 2 * Va / L_w
        w_b0 = (w_b1 / 2)**2

        # Dryden transfer functions (section 4.4 UAV book) - Fill in proper num and den
        self.u_w = TransferFunction(num=np.array([[u_a0]]), den=np.array([[u_b1,u_b0]]),Ts=Ts)
        self.v_w = TransferFunction(num=np.array([[v_a1,v_a0]]), den=np.array([[v_b2,v_b1,v_b0]]),Ts=Ts)
        self.w_w = TransferFunction(num=np.array([[w_a1,w_a0]]), den=np.array([[w_b2,w_b1,w_b0]]),Ts=Ts)
        self._Ts = Ts

    def update(self):
        # returns a six vector.
        #   The first three elements are the steady state wind in the inertial frame
        #   The second three elements are the gust in the body frame
        gust = np.array([[self.u_w.update(np.random.randn())],
                         [self.v_w.update(np.random.randn())],
                         [self.w_w.update(np.random.randn())]])
        # gust = np.array([[0.],[0.],[0.]])
        return np.concatenate(( self._steady_state, gust ))

