"""
mavDynamics 
    - this file implements the dynamic equations of motion for MAV
    - use unit quaternion for the attitude state
    
mavsim_python
    - Beard & McLain, PUP, 2012
    - Update history:  
        2/24/2020 - RWB
"""
import numpy as np
from models.mav_dynamics import MavDynamics as MavDynamicsForces
# load message types
from message_types.msg_state import MsgState
from message_types.msg_delta import MsgDelta
import parameters.aerosonde_parameters as MAV
from tools.rotations import quaternion_to_rotation, quaternion_to_euler
import math

class MavDynamics(MavDynamicsForces):
    def __init__(self, Ts):
        super().__init__(Ts)
        # store wind data for fast recall since it is used at various points in simulation
        self._wind = np.array([[0.], [0.], [0.]])  # wind in NED frame in meters/sec
        # store forces to avoid recalculation in the sensors function
        self._forces = np.array([[0.], [0.], [0.]])
        self._Va = MAV.Va0
        self._alpha = 0
        self._beta = 0
        # update velocity data and forces and moments
        self._update_velocity_data()
        self._forces_moments(delta=MsgDelta())
        # update the message class for the true state
        self._update_true_state()


    ###################################
    # public functions
    def update(self, delta, wind):
        '''
            Integrate the differential equations defining dynamics, update sensors
            delta = (delta_a, delta_e, delta_r, delta_t) are the control inputs
            wind is the wind vector in inertial coordinates
            Ts is the time step between function calls.
        '''
        # get forces and moments acting on rigid bod
        forces_moments = self._forces_moments(delta)
        super()._rk4_step(forces_moments)
        # update the airspeed, angle of attack, and side slip angles using new state
        self._update_velocity_data(wind)
        # update the message class for the true state
        self._update_true_state()

    ###################################
    # private functions
    def _update_velocity_data(self, wind=np.zeros((6,1))):
        steady_state = wind[0:3]
        gust = wind[3:6]
        ##### TODO #####
        # convert steady-state wind vector from world to body frame
        # phi, theta, psi = quaternion_to_euler(self._state[6:10])
        # wind_body = R_vb @ steady_state
        # R_vb = np.array([[np.cos(theta)*np.cos(psi), np.cos(theta)*np.sin(psi), -np.sin(theta)],
        #                 [np.sin(phi)*np.sin(theta)*np.cos(psi) - np.cos(phi)*np.sin(psi), np.sin(phi)*np.sin(theta)*np.sin(psi) + np.cos(phi)*np.cos(psi), np.sin(phi)*np.cos(theta)],
        #                 [np.cos(phi)*np.sin(theta)*np.cos(psi) + np.sin(phi)*np.sin(psi), np.cos(phi)*np.sin(theta)*np.sin(psi) - np.sin(phi)*np.cos(psi), np.cos(phi)*np.cos(theta)]])
        R_b2i = quaternion_to_rotation(self._state[6:10])
        wind_body = R_b2i.T @ steady_state
        # add the gust 
        wind_body += gust
        # convert total wind to world frame
        # self._wind = R_b2i @ wind_body
        self._wind = wind_body

        # V_a^b - velocity vector relative to the airmass ([ur , vr, wr]= ?)
        ur = self._state[3, 0] - self._wind[0, 0]
        vr = self._state[4, 0] - self._wind[1, 0]
        wr = self._state[5, 0] - self._wind[2, 0]

        # compute airspeed (self._Va = ?)
        # self._Va = np.linalg.norm([ur, vr, wr])
        self._Va = np.sqrt(ur ** 2 + vr ** 2 + wr ** 2)
        # compute angle of attack (self._alpha = ?)
        self._alpha = np.arctan2(wr, ur)
        # compute sideslip angle (self._beta = ?)
        # self._beta = np.arctan2(vr, np.sqrt(ur**2 + vr**2))
        self._beta = np.arcsin(vr/self._Va) 
        """ 就这个侧滑角和check相差0.0001左右 """
    def _forces_moments(self, delta):
        """
        return the forces on the UAV based on the state, wind, and control surfaces
        :param delta: np.matrix(delta_a, delta_e, delta_r, delta_t)
        :return: Forces and Moments on the UAV np.matrix(Fx, Fy, Fz, Ml, Mn, Mm)
        """
        ##### TODO ######
        # extract states (phi, theta, psi, p, q, r)
        e = self._state[6:10]
        # eularangle = quaternion_to_euler(e)
        # phi = eularangle[0]
        # theta = eularangle[1]
        # psi = eularangle[2]
        p = self._state.item(10)
        q = self._state.item(11)
        r = self._state.item(12)
        # compute gravitational forces ([fg_x, fg_y, fg_z])
        fg = MAV.mass * MAV.gravity * np.array([[2. * (e[1,0]*e[3,0] - e[2,0]*e[0,0])], 
                                                [2. * (e[2,0]*e[3,0] + e[1,0]*e[0,0])], 
                                                [e[3,0]**2. + e[0,0]**2. - e[1,0]**2. - e[2,0]**2.]])
        # compute Lift and Drag coefficients (CL-Slide17, CD-Slide20)
        a1 = np.exp(-MAV.M * (self._alpha - MAV.alpha0))
        a2 = np.exp(MAV.M * (self._alpha + MAV.alpha0))
        sigma_alpha = (1 + a1 + a2) / ((1 + a1) * (1 + a2))
        CL = (1 - sigma_alpha) * (MAV.C_L_0 + MAV.C_L_alpha * self._alpha) + \
            sigma_alpha * (2 * np.sign(self._alpha) * np.sin(self._alpha)**2 * np.cos(self._alpha))
        CD = MAV.C_D_p + (MAV.C_L_0 + MAV.C_L_alpha * self._alpha)**2 / np.pi / MAV.e_os / MAV.AR
        # compute Lift and Drag Forces (F_lift, F_drag-Slide22)
        rhoVa2S_2 = 0.5 * MAV.rho * self._Va**2 * MAV.S_wing
        F_lift = rhoVa2S_2 * (CL + MAV.C_L_q * MAV.c / (2*self._Va)*q + \
                              MAV.C_L_delta_e*delta.elevator)
        F_drag = rhoVa2S_2 * (CD + MAV.C_D_q * MAV.c / (2*self._Va)*q + \
                              MAV.C_D_delta_e*delta.elevator)
        # propeller thrust and torque
        thrust_prop, torque_prop = self._motor_thrust_torque(self._Va, delta.throttle)

        # compute longitudinal forces in body frame (fx, fz-Slide43)
        c_alpha = np.cos(self._alpha)
        s_alpha = np.sin(self._alpha)
        fx = fg[0,0] + c_alpha * (-F_drag) + (-s_alpha) * (-F_lift) + thrust_prop
        fz = fg[2,0] + s_alpha * (-F_drag) + (c_alpha) * (-F_lift)
        # compute lateral forces in body frame (fy)
        fy = fg[1,0] + rhoVa2S_2 * (MAV.C_Y_0 + MAV.C_Y_beta * self._beta + \
                                    MAV.C_Y_p * MAV.b / (2*self._Va) * p + \
                                    MAV.C_Y_r * MAV.b / (2*self._Va) * r + \
                                    MAV.C_Y_delta_a * delta.aileron + \
                                    MAV.C_Y_delta_r * delta.rudder)
        # compute logitudinal torque in body frame (My)
        m = rhoVa2S_2 * MAV.c * (MAV.C_m_0 + MAV.C_m_alpha * self._alpha + \
                                 MAV.C_m_q * MAV.c / (2*self._Va) * q + \
                                 MAV.C_m_delta_e * delta.elevator)
        # compute lateral torques in body frame (Mx, Mz)
        l = rhoVa2S_2 * MAV.b * (MAV.C_ell_0 + MAV.C_ell_beta * self._beta + \
                                 MAV.C_ell_p * MAV.b / (2*self._Va) * p + \
                                 MAV.C_ell_r * MAV.b / (2*self._Va) * r + \
                                 MAV.C_ell_delta_a * delta.aileron + \
                                 MAV.C_ell_delta_r * delta.rudder) - torque_prop
        n = rhoVa2S_2 * MAV.b * (MAV.C_n_0 + MAV.C_n_beta * self._beta + \
                                 MAV.C_n_p * MAV.b / (2*self._Va) * p + \
                                 MAV.C_n_r * MAV.b / (2*self._Va) * r + \
                                 MAV.C_n_delta_a * delta.aileron + \
                                 MAV.C_n_delta_r * delta.rudder)
        forces_moments = np.array([[fx, fy, fz, l, m, n]]).T
        # forces_moments = np.array([[0, 0, 0, 0, 0, 0]]).T
        # print("forces_moments: ", forces_moments)
        return forces_moments

    def _motor_thrust_torque(self, Va, delta_t):
        # compute thrust and torque due to propeller
        ##### TODO #####
        # map delta_t throttle command(0 to 1) into motor input voltage
        v_in = MAV.V_max * delta_t
        # Angular speed of propeller (omega_p = ?)
        a = MAV.rho * (MAV.D_prop**5.0) * MAV.C_Q0 / (2.0*np.pi)**2.0
        b = MAV.rho * (MAV.D_prop**4.0) * MAV.C_Q1 * Va / 2.0 / np.pi + MAV.KQ * MAV.KV / MAV.R_motor
        c = MAV.rho * (MAV.D_prop**3.0) * MAV.C_Q2 * Va**2.0 - MAV.KQ * v_in / MAV.R_motor + MAV.KQ * MAV.i0
        omega_op = (-b + np.sqrt(b**2.0 - 4.0*a*c)) / (2.0*a)
        J_op = 2.0 * np.pi * Va / omega_op / MAV.D_prop
        # thrust and torque due to propeller
        thrust_prop = ((MAV.rho * (MAV.D_prop ** 4.0) * MAV.C_T0) / (4.0 * (np.pi ** 2.0))) * (omega_op ** 2.0) + \
                      ((MAV.rho * (MAV.D_prop ** 3.0) * MAV.C_T1 * Va) / (2.0 * np.pi)) * omega_op + \
                      MAV.rho * (MAV.D_prop ** 2.0) * MAV.C_T2 * (Va ** 2.0)
        torque_prop = ((MAV.rho * (MAV.D_prop ** 5.0) * MAV.C_Q0) / (4.0 * (np.pi ** 2.0))) * (omega_op ** 2.0) + \
                      ((MAV.rho * (MAV.D_prop ** 4.0) * MAV.C_Q1 * Va) / (2.0 * np.pi)) * omega_op + \
                      MAV.rho * (MAV.D_prop ** 3.0) * MAV.C_Q2 * (Va ** 2.0)
        # C_T = MAV.C_T2 * J_op**2.0 + MAV.C_T1 * J_op + MAV.C_T0
        # C_Q = MAV.C_Q2 * J_op**2.0 + MAV.C_Q1 * J_op + MAV.C_Q0
        # thrust_prop = MAV.rho * (omega_op / 2.0 / np.pi)**2 * MAV.D_prop**4 * C_T
        # torque_prop = MAV.rho * (omega_op / 2.0 / np.pi)**2 * MAV.D_prop**5 * C_Q
        
        # thrust_prop = torque_prop = 0
        return thrust_prop, torque_prop

    def _update_true_state(self):
        # rewrite this function because we now have more information
        phi, theta, psi = quaternion_to_euler(self._state[6:10])
        pdot = quaternion_to_rotation(self._state[6:10]) @ self._state[3:6]
        self.true_state.north = self._state.item(0)
        self.true_state.east = self._state.item(1)
        self.true_state.altitude = -self._state.item(2)
        self.true_state.Va = self._Va
        self.true_state.alpha = self._alpha
        self.true_state.beta = self._beta
        self.true_state.phi = phi
        self.true_state.theta = theta
        self.true_state.psi = psi
        self.true_state.Vg = np.linalg.norm(pdot)
        self.true_state.gamma = np.arcsin(pdot.item(2) / self.true_state.Vg)
        self.true_state.chi = np.arctan2(pdot.item(1), pdot.item(0))
        self.true_state.p = self._state.item(10)
        self.true_state.q = self._state.item(11)
        self.true_state.r = self._state.item(12)
        self.true_state.wn = self._wind.item(0)
        self.true_state.we = self._wind.item(1)
        self.true_state.bx = 0
        self.true_state.by = 0
        self.true_state.bz = 0
        self.true_state.camera_az = 0
        self.true_state.camera_el = 0
