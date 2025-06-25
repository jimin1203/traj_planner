# main_offline_callback 함수 코드 분석

## Import 및 의존성

```python
import numpy as np
import configparser
import pickle
import os.path as osfuncs
import hashlib
import logging

# custom modules
import graph_ltpl
```

## MD5 해시 함수

```python
    new_base_generated = False
    graph_base = None

    calculated_md5 = md5(globtraj_param_path) + md5(graph_off_config_path)
```

- MD5 해시 계산. 파일의 변경 여부를 확인하는데 사용. 변경되면, 새 그래프 생성.
- get the MD5-hash of all config files to recalculate whenever any file changed

## 기존 그래프가 있는지 확인

```python
# 기존 그래프가 있다면 load
    if not force_recalc and osfuncs.isfile(graph_store_path):
        f = open(graph_store_path, 'rb')
        graph_base = pickle.load(f)
        f.close()
        logging.getLogger("local_trajectory_logger").debug("Loaded database with " + str(len(graph_base.get_nodes())) + " node and " + str(len(graph_base.get_edges())) + " edges from file...")
```

- 이전에 저장한 그래프(.pickle)이 있고 `force_recalc`이 False라면 저장된 그래프를 불러온다.

## (분기점) 새 그래프 생성 조건 확인 및 생성
```python
    if force_recalc or graph_base is None or calculated_md5 != graph_base.md5_params:
        new_base_generated = True
        if force_recalc:
            print("Manually forced recalculation of graph! Skipped graph import from file!")
        if graph_base is not None and calculated_md5 is not graph_base.md5_params:
            print("MD5-Sum of any param-file does not match the one in the graph object! Triggered recalculation!")

        # 그래프 설정 파일 로드
        graph_config = configparser.ConfigParser()
        if not graph_config.read(graph_off_config_path):
            raise ValueError('Specified graph config file does not exist or is empty!')
```
- `force_recalc`= True(강제 재계산 On), `graph_base`가 있거나, param 해시가 불일치할 때 new graph를 생성한다.
- ConfigParser()를 이용하여 `graph_config`라는 .ini를 읽어주는 도우미를 정의한다.

## Global Trajectory Data 읽기
```python 
        refline, t_width_right, t_width_left, normvec_normalized, alpha, length_rl, vel_rl, kappa_rl \
        = graph_ltpl.imp_global_traj.src.import_globtraj_csv.import_globtraj_csv(import_path=globtraj_param_path)

        s = np.concatenate(([0], np.cumsum(length_rl)))
        xy = refline + normvec_normalized * alpha[:, np.newaxis]
        raceline_params = np.column_stack((xy, kappa_rl, vel_rl))
```
- 경로 관련 데이터를 CSV file로부터 읽는다.
- 읽은 데이터를 이용하여 s(누적 거리), xy(raceline 좌표), 여러 params을 변수에 저장.

**[import_globtraj_csv](import_globtraj_csv.md)**

```python 
        closed = (np.hypot(xy[0, 0] - xy[-1, 0], xy[0, 1] - xy[-1, 1])
                  < graph_config.getfloat('LATTICE', 'closure_detection_dist'))

        if closed:
            logging.getLogger("local_trajectory_logger").debug("Input line is interpreted as closed track!")
            glob_rl = np.column_stack((s, np.vstack((raceline_params, raceline_params[0, :]))))
            
        else:
            logging.getLogger("local_trajectory_logger").debug("Input line is interpreted as _unclosed_ track!")
            glob_rl = np.column_stack((s[:-1], raceline_params))
```
- 경로가 폐곡선인지 확인
  
## 곡률에 따라 node의 위치를 선정
```python 
        idx_array = graph_ltpl.imp_global_traj.src.variable_step_size. \
            variable_step_size(kappa=kappa_rl,
                               dist=length_rl,
                               d_curve=graph_config.getfloat('LATTICE', 'lon_curve_step'),
                               d_straight=graph_config.getfloat('LATTICE', 'lon_straight_step'),
                               curve_th=graph_config.getfloat('LATTICE', 'curve_thr'),
                               force_last=not closed)
```
- 곡률구간이면 간격이 더 촘촘. 직선구간이면 간격이 널널.

**[variable_step_size](variable_step_size.md)**
  
## 선택된 index(레이어)에 따라 각 데이터를 추출
```python
        refline = refline[idx_array, :]
        t_width_right = t_width_right[idx_array]
        t_width_left = t_width_left[idx_array]
        normvec_normalized = normvec_normalized[idx_array]
        alpha = alpha[idx_array]
        vel_rl = vel_rl[idx_array]
        s_raceline = s[idx_array]

        length_rl_tmp = []
        for idx_from, idx_to in zip(idx_array[:-1], idx_array[1:]):
            length_rl_tmp.append(np.sum(length_rl[idx_from:idx_to]))

        length_rl_tmp.append(0.0)
        length_rl = list(length_rl_tmp)
```
- layer 간격을 정했으므로(샘플링을 다시 한 것과 같다.) 이에 따라, refline 재정의 
  - `idx_array`에 해당하는 행만 추출. (python list면 불가하나 np.array이기 때문에 문법적으로 ok)
- 노드간 거리(length_rl) 재계산
  - 인접한 두 노드(샘플링 후) 간 거리를 구한다.
    - idx_from 이상 idx_to 미만 구간의 모든 원소를 선택하여 sum

## GraphBase 객체 생성
```python 
        graph_base = graph_ltpl.data_objects.GraphBase.\
            GraphBase(lat_off   =graph_config.getfloat('LATTICE', 'lat_offset'),
                      num_layers=np.size(alpha, axis=0),
                      refline=refline,
                      normvec_normalized=normvec_normalized,
                      track_width_right=t_width_right,
                      track_width_left=t_width_left,
                      alpha=alpha,
                      vel_raceline=vel_rl,
                      s_raceline=s_raceline,
                      lat_resolution=graph_config.getfloat('LATTICE', 'lat_resolution'),
                      sampled_resolution=graph_config.getfloat('SAMPLING', 'stepsize_approx'),
                      vel_decrease_lat=graph_config.getfloat('PLANNINGTARGET', 'vel_decrease_lat'),
                      veh_width=graph_config.getfloat('VEHICLE', 'veh_width'),
                      veh_length=graph_config.getfloat('VEHICLE', 'veh_length'),
                      veh_turn=graph_config.getfloat('VEHICLE', 'veh_turn'),
                      md5_params=calculated_md5,
                      graph_id=graph_id,
                      glob_rl=glob_rl,
                      virt_goal_node=graph_config.getboolean('LATTICE', 'virt_goal_n'),
                      virt_goal_node_cost=graph_config.getfloat('COST', 'w_virt_goal'),
                      min_plan_horizon=graph_config.getfloat('PLANNINGTARGET', 'min_plan_horizon'),
                      plan_horizon_mode=graph_config.get('PLANNINGTARGET', 'plan_horizon_mode'),
                      closed=closed)
```
- 위에서 준비한 값들을 이용하여 GraphBase 객체를 생성

## node skeleton 생성
```python 
        state_pos = graph_ltpl.offline_graph.src.gen_node_skeleton. \
            gen_node_skeleton(graph_base=graph_base,
                              length_raceline=length_rl,
                              var_heading=graph_config.getboolean('LATTICE', 'variable_heading'))
```
- 노드들의 실제 [x, y]와 heading 등 생성

**[gen_node_skeleton](gen_node_skeleton.md)**

## Edge 생성
```python 
        state_pos_arr = np.empty(shape=(len(state_pos), 2), dtype=np.object)
        state_pos_arr[:] = state_pos

        graph_ltpl.offline_graph.src.gen_edges.gen_edges(state_pos=state_pos_arr,
                                                         graph_base=graph_base,
                                                         stepsize_approx=graph_config.getfloat('SAMPLING',
                                                                                               'stepsize_approx'),
                                                         min_vel_race=graph_config.getfloat('LATTICE', 'min_vel_race'),
                                                         closed=closed)
```
- 실제 경로상의 연결, 속도, 곡선 등 다양한 조건을 고려하여 Edge를 생성한다.

**[gen_edges](gen_edges.md)**

## prune Graph
```python 
        graph_ltpl.offline_graph.src.prune_graph.prune_graph(graph_base=graph_base,
                                                             closed=closed)
```
- 도달 불가능한 노드와 엣지를 제거 
  - 쓸모없는(dead-end) 경로를 정리 
  
**[prune_graph](prune_graph.md)**

## Offline Cost 계산
```python 
        graph_ltpl.offline_graph.src.gen_offline_cost.gen_offline_cost(graph_base=graph_base,
                                                                       cost_config_path=graph_off_config_path)
```

**[gen_offline_cost](gen_offline_cost.md)**

## 그래프 필터링 및 graph를 .pickle로 저장
```python 
        graph_base.init_filtering()

        # store graph for later use
        f = open(graph_store_path, 'wb')
        pickle.dump(graph_base, f)
        f.close()
```

## (분기점) 
```python 
    else:
        if graph_logging_path is not None:
            graph_logging_path = (graph_logging_path[:graph_logging_path.rfind('/Graph_Objects/') + 15]
                                  + str(graph_base.graph_id) + ".pckl")
```
    - 이미 존재하는 graph가 있어서 load한다.

## log_path가 따로 주어지면 별도로 저장
```python 
    if graph_logging_path is not None and not osfuncs.isfile(graph_logging_path):
        f = open(graph_logging_path, 'wb')
        pickle.dump(graph_base, f)
        f.close()
```

```python 
    return graph_base, new_base_generated
```

