import sys
import os

# -- Limit number of OPENBLAS library threads --
# On linux based operation systems, we observed a occupation of all cores by the underlying openblas library. Often,
# this slowed down other processes, as well as the planner itself. Therefore, it is recommended to set the number of
# threads to one. Note: this import must happen before the import of any openblas based package (e.g. numpy)
os.environ['OPENBLAS_NUM_THREADS'] = str(1) # 과도한 연산을 막기 위하여 한 번에 하나의 스레드만 사용하도록 설정

import numpy as np
import json
import time
import configparser

import graph_ltpl

"""
This is the main script to run a minimal example of the graph-based local trajectory planner. The minimal example
generates an offline graph for a race track and drives around the track without the interface to an object list.
Furthermore, logs and advanced vehicle dynamics are not considered.

:Authors:
    * Tim Stahl <tim.stahl@tum.de>

:Created on:
    06.05.2020
"""

# ----------------------------------------------------------------------------------------------------------------------
# IMPORT (should not change) -------------------------------------------------------------------------------------------
# ----------------------------------------------------------------------------------------------------------------------
# top level path (module directory)
toppath = os.path.dirname(os.path.realpath(__file__)) 
sys.path.append(toppath)
print(toppath)
track_param = configparser.ConfigParser() # configparser는 .ini 파일을 읽기 위한 모듈 
if not track_param.read (toppath + "/params/driving_task.ini"): # [Drving Task]
    raise ValueError('Specified online parameter config file does not exist or is empty!')

track_specifier = json.loads(track_param.get('DRIVING_TASK', 'track'))

# define all relevant paths
path_dict = {'globtraj_input_path': toppath + "/inputs/traj_ltpl_cl/traj_ltpl_cl_" + track_specifier + ".csv",
             'graph_store_path': toppath + "/inputs/stored_graph.pckl", # 안 쓰임.
             'ltpl_offline_param_path': toppath + "/params/ltpl_config_offline.ini",
             'ltpl_online_param_path': toppath + "/params/ltpl_config_online.ini"
             }

# ----------------------------------------------------------------------------------------------------------------------
# INITIALIZATION AND OFFLINE PART --------------------------------------------------------------------------------------
# ----------------------------------------------------------------------------------------------------------------------

# intialize graph_ltpl-class
ltpl_obj = graph_ltpl.Graph_LTPL.Graph_LTPL(path_dict=path_dict,
                                            visual_mode=True,
                                            log_to_file=False)

# calculate offline graph
ltpl_obj.graph_init() # ltpl_obj.graph_init()는 그래프를 초기화하는 함수

# set start pose based on first point in provided reference-line
refline = graph_ltpl.imp_global_traj.src.\
    import_globtraj_csv.import_globtraj_csv(import_path=path_dict['globtraj_input_path'])[0]
pos_est = refline[0, :]
heading_est = np.arctan2(np.diff(refline[0:2, 1]), np.diff(refline[0:2, 0])) - np.pi / 2
vel_est = 0.0

# set start pos
ltpl_obj.set_startpos(pos_est=pos_est,
                      heading_est=heading_est)

# ----------------------------------------------------------------------------------------------------------------------
# ONLINE LOOP ----------------------------------------------------------------------------------------------------------
# ----------------------------------------------------------------------------------------------------------------------
# 계획 -> 이동 -> 상태 갱신 -> 다시 계획 반복
traj_set = {'straight': None}
tic = time.time()

while True:
    # -- SELECT ONE OF THE PROVIDED TRAJECTORIES -----------------------------------------------------------------------
    # (here: brute-force, replace by sophisticated behavior planner)
    for sel_action in ["right", "left", "straight", "follow"]:  # try to force 'right', else try next in list(우선순위)
        if sel_action in traj_set.keys():
            break

    # -- CALCULATE PATHS FOR NEXT TIMESTAMP ----------------------------------------------------------------------------
    ltpl_obj.calc_paths(prev_action_id=sel_action,
                        object_list=[])
    print(f"traj_set[sel_action]: {sel_action}, {traj_set[sel_action]}")
    # -- GET POSITION AND VELOCITY ESTIMATE OF EGO-VEHICLE -------------------------------------------------------------
    # (here: simulation dummy, replace with actual sensor readings)
    # 시뮬레이션이기 때문에 실제 센서가 아닌 시뮬레이션으로 차량의 위치와 속도를 추정
    # 실제 차량에서는 센서로부터 받아온다.
    if traj_set[sel_action] is not None: # 경로가 존재할 때
        pos_est, vel_est = graph_ltpl.testing_tools.src.vdc_dummy.\
            vdc_dummy(pos_est=pos_est, 
                      last_s_course=(traj_set[sel_action][0][:, 0]),
                      last_path=(traj_set[sel_action][0][:, 1:3]),
                      last_vel_course=(traj_set[sel_action][0][:, 5]),
                      iter_time=time.time() - tic) # 다음 위치와 속도를 가상으로 계산해서 반환
    tic = time.time()

    # -- CALCULATE VELOCITY PR
    # OFILE AND RETRIEVE TRAJECTORIES ----------------------------------------------------------
    traj_set = ltpl_obj.calc_vel_profile(pos_est=pos_est,
                                         vel_est=vel_est)[0]
    
    # -- LIVE PLOT (if activated) --------------------------------------------------------------------------------------
    ltpl_obj.visual()
