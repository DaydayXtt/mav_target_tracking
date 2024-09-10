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
from message_types.msg_sensors import MsgSensors
import parameters.aerosonde_parameters as MAV
import parameters.sensor_parameters as SENSOR
from models.mav_dynamics_control import MavDynamics as MavDynamicsNoSensors
from tools.rotations import quaternion_to_rotation, quaternion_to_euler, euler_to_rotation

class MavDynamics(MavDynamicsNoSensors):
    def __init__(self, Ts):
        super().__init__(Ts)
        # initialize the sensors message
        self._sensors = MsgSensors()
        # random walk parameters for GPS
        self._gps_eta_n = 0.
        self._gps_eta_e = 0.
        self._gps_eta_h = 0.
        # timer so that gps only updates every ts_gps seconds
        self._t_gps = 999.  # large value ensures gps updates at initial time.

    def sensors(self):
        "Return value of sensors on MAV: gyros, accels, absolute_pressure, dynamic_pressure, GPS"
        # true value
        north = self._state.item(0)
        east = self._state.item(1)
        h = - self._state.item(2)  # h = -pd
        p = self._state.item(10)
        q = self._state.item(11)
        r = self._state.item(12)
        phi, theta, psi = quaternion_to_euler(self._state[6:10])
        # pdot =

        # simulate rate gyros(units are rad / sec)
        self._sensors.gyro_x = p + np.random.normal(loc=0, scale=SENSOR.gyro_sigma) + SENSOR.gyro_x_bias
        self._sensors.gyro_y = q + np.random.normal(loc=0, scale=SENSOR.gyro_sigma) + SENSOR.gyro_y_bias
        self._sensors.gyro_z = r + np.random.normal(loc=0, scale=SENSOR.gyro_sigma) + SENSOR.gyro_z_bias

        # simulate accelerometers(units of g)
        # self._forces: x,y,z
        self._sensors.accel_x = self._forces.item(0) / MAV.mass + MAV.gravity * np.sin(theta) + \
                                np.random.normal(loc=0, scale=SENSOR.accel_sigma)
        self._sensors.accel_y = self._forces.item(1) / MAV.mass - MAV.gravity * np.cos(theta) * np.sin(phi) + \
                                np.random.normal(loc=0, scale=SENSOR.accel_sigma)
        self._sensors.accel_z = self._forces.item(2) / MAV.mass - MAV.gravity * np.cos(theta) * np.cos(phi) + \
                                np.random.normal(loc=0, scale=SENSOR.accel_sigma)
        # simulate magnetometers
        # magnetic field in provo has magnetic declination of 12.5 degrees
        # and magnetic inclination of 66 degrees
        # Beijing: declination: -9.0 degrees, inclination: 57.5 degrees.
        # From https://www.magnetic-declination.com/China/Beijing/388314.html
        R_mag = euler_to_rotation(0, np.radians(57.5), np.radians(-9.0))
        # magnetic field in inertial frame: unit vector
        mag_inertial = R_mag @ np.array([[1], [0], [0]])
        R = euler_to_rotation(phi, theta, psi)  # body to inertial, need to inverse.
        # magnetic field in body frame: unit vector
        mag_body = R.T @ mag_inertial
        self._sensors.mag_x = mag_body.item(0) + SENSOR.mag_beta + np.random.normal(loc=0, scale=SENSOR.mag_sigma)
        self._sensors.mag_y = mag_body.item(1) + SENSOR.mag_beta + np.random.normal(loc=0, scale=SENSOR.mag_sigma)
        self._sensors.mag_z = mag_body.item(2) + SENSOR.mag_beta + np.random.normal(loc=0, scale=SENSOR.mag_sigma)

        # simulate pressure sensors
        self._sensors.abs_pressure = MAV.rho * MAV.gravity * (h - 0) + \
                                     np.random.normal(loc=0, scale=SENSOR.abs_pres_sigma)
        self._sensors.diff_pressure = MAV.rho * (self._Va ** 2.) / 2. + \
                                      np.random.normal(loc=0, scale=SENSOR.diff_pres_sigma)
        # simulate GPS sensor
        if self._t_gps >= SENSOR.ts_gps:
            coe_1 = np.exp(-SENSOR.gps_k * SENSOR.ts_gps)
            self._gps_eta_n = coe_1 * self._gps_eta_n + np.random.normal(loc=0, scale=SENSOR.gps_n_sigma)
            self._gps_eta_e = coe_1 * self._gps_eta_e + np.random.normal(loc=0, scale=SENSOR.gps_e_sigma)
            self._gps_eta_h = coe_1 * self._gps_eta_h + np.random.normal(loc=0, scale=SENSOR.gps_h_sigma)
            self._sensors.gps_n = north + self._gps_eta_n
            self._sensors.gps_e = east + self._gps_eta_e
            self._sensors.gps_h = h + self._gps_eta_h
            # wind: N E D
            self._sensors.gps_Vg = (np.sqrt(((self._Va * np.cos(psi) + self._wind[0]) ** 2)
                                            + ((self._Va * np.sin(psi) + self._wind[1]) ** 2)) +
                                    np.random.normal(loc=0, scale=SENSOR.gps_Vg_sigma)).item()
            self._sensors.gps_course = \
                (np.arctan2(self._Va * np.sin(psi) + self._wind[1], self._Va * np.cos(psi) + self._wind[0]) +
                 np.random.normal(loc=0, scale=SENSOR.gps_course_sigma)).item()
            self._t_gps = 0.
        else:
            self._t_gps += self._ts_simulation
        return self._sensors

    def external_set_state(self, new_state):
        self._state = new_state

    def _update_true_state(self):
        # update the class structure for the true state:
        #   [pn, pe, h, Va, alpha, beta, phi, theta, chi, p, q, r, Vg, wn, we, psi, gyro_bx, gyro_by, gyro_bz]
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
        self.true_state.bx = SENSOR.gyro_x_bias
        self.true_state.by = SENSOR.gyro_y_bias
        self.true_state.bz = SENSOR.gyro_z_bias
        self.true_state.camera_az = self._state.item(13)
        self.true_state.camera_el = self._state.item(14)