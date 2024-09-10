"""
mavsim_python
    - Chapter 10 assignment for Beard & McLain, PUP, 2012
    - Last Update:
        3/11/2019 - RWB
        2/27/2020 - RWB
        1/5/2023 - David L. Christiansen
        7/13/2023 - RWB
"""
import os, sys
# insert parent directory at beginning of python search path
from pathlib import Path
sys.path.insert(0,os.fspath(Path(__file__).parents[2]))
# use QuitListener for Linux or PC <- doesn't work on Mac
from tools.quit_listener import QuitListener
import numpy as np
import parameters.simulation_parameters as SIM
from models.mav_dynamics_sensors import MavDynamics
from models.wind_simulation import WindSimulation
from controllers.autopilot import Autopilot
# from estimators.observer_full import Observer
# from estimators.observer import Observer
from planners.path_follower import PathFollower
from planners.path_manager_follow_target import PathManager
from viewers.view_manager import ViewManager
from message_types.msg_path import MsgPath
import time

# initialize elements of the architecture
wind = WindSimulation(SIM.ts_simulation)
mav = MavDynamics(SIM.ts_simulation)
autopilot = Autopilot(SIM.ts_simulation)
# observer = Observer(SIM.ts_simulation)
path_follower = PathFollower()
path_manager = PathManager()

viewers = ViewManager(path=True,animation=True, 
                      data=True,save_plots=True,
                      video=True, video_name='TT.mp4')
quitter = QuitListener()

# path definition
# path = MsgPath(
#     type ='line',
#     airspeed = 40,
#     line_origin = np.array([[0.0, 0.0, -100.0]]).T,
#     line_direction = np.array([[-500, 0, 0.0]]).T,
#     )

# initialize the simulation time
sim_time = SIM.start_time
end_time = 50

# main simulation loop
print("Press 'Esc' to exit...")
cur_goal = np.array([[-100. + np.random.uniform(0, 200), 
                      -100. + np.random.uniform(0, 200), 
                      -100. + np.random.uniform(0, 20)]]).T
while sim_time < end_time:
    # current goal
    if np.linalg.norm(mav._state[0:3] - cur_goal) < 50:
        # 在200*200内随机生成一个目标点
        cur_goal = np.array([[-100. + np.random.uniform(0, 200), 
                            -100. + np.random.uniform(0, 200), 
                            -100. + np.random.uniform(0, 50)]]).T
    # cur_goal = cur_goal + np.array([[-0.01, 0.05, -0.05]]).T
    path = path_manager.update(cur_goal)

    # -------observer-------------
    # measurements = mav.sensors()  # get sensor measurements
    # estimated_state = observer.update(measurements)  # estimate states from measurements
    estimated_state = mav.true_state
    # -------path follower-------------
    autopilot_commands = path_follower.update(path, estimated_state)

    # -------autopilot-------------
    delta, commanded_state = autopilot.update(autopilot_commands, estimated_state)

    # -------physical system-------------
    current_wind = wind.update()  # get the new wind vector
    mav.update(delta, current_wind)  # propagate the MAV dynamics

    # -------- update viewer -------------
    viewers.update(
        sim_time,
        true_state=mav.true_state,  # true states
        # estimated_state=estimated_state,  # estimated states        
        commanded_state=commanded_state,  # commanded states
        delta=delta, # inputs to MAV
        path=path, # path
        target_position=cur_goal, # target position
    )
        
    # -------Check to Quit the Loop-------
    if quitter.check_quit():
        break

    # -------increment time-------------
    sim_time += SIM.ts_simulation
    # time.sleep(0.005)  # make the sim run slower

# close viewers
viewers.close(dataplot_name="TT_data_plot")





