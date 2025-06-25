### OnlineTrajectoryHandler/calc_paths
사실상 calc paths의 main 함수

```python
    def calc_paths(self,
                   action_id_sel: str,
                   idx_sel_traj: int) -> tuple:
```

```python
        if action_id_sel == 'emergency':
            action_id_sel = self.__em_base_id

        # if action id is forced (e.g. init)
        if self.__action_id_forced is not None:
            action_id_sel = self.__action_id_forced
            self.__action_id_forced = None
```

```python
            const_path_seg_exists = (self.__last_action_set_path_param is not None
                                 and action_id_sel in self.__last_action_set_path_param.keys()) # 이전 경로가 유효한지.
        planned_once = self.__last_path_timestamp is not None
        valid_solution_last_step = (planned_once and const_path_seg_exists
                                    and self.__last_bp_action_set[action_id_sel][idx_sel_traj].shape[0] > 2)
```
- `const_path_seg_exists`: 이전 경로가 유효한지 확인한다. 그래서 last_action_set_path_param에서 지금 선택된 action id가 있는지 확인.
- `planned_once`: 첫번째 plan인지 아닌지 확인 (필요한 변수인지를 확인해봐야할 듯)
- `valid_solution_last_step`: 맨 처음에는 false가 된다.

```python
        if valid_solution_last_step:
            temp_id = "straight"
            if "follow" in self.__last_action_set_nodes.keys():
                temp_id = "follow"
            # 유효한 경로가 존재할 때, 맨 앞에 있는 것만 추출해냄.
            self.__backup_coeff = self.__last_action_set_coeff[temp_id][0]
            self.__backup_node_idx = self.__last_action_set_node_idx[temp_id][0]
            self.__backup_nodes = self.__last_action_set_nodes[temp_id][0]
            self.__backup_path_param = self.__last_action_set_path_param[temp_id][0]
            self.__backup_path_gg = self.__last_action_set_path_gg[temp_id][0]
        else:
            self.__backup_coeff = None
            self.__backup_node_idx = None
            self.__backup_nodes = None
            self.__backup_path_param = None
            self.__backup_path_gg = None
```
- 이전 solution이 존재할 때, 이미 계산된 경로의 모든 파라미터를 저장해둔다.

### (if)두 번째 planning 단계부터 적용되는 부분 

```python
        if planned_once and valid_solution_last_step: 
            # extract calculation time for last iteration
            calc_time = time.time() - self.__last_path_timestamp # 차량이 이전 경로-현재까지 이동한 시간
            self.__last_path_timestamp = time.time()

            # warn if calc time exceeds threshold
            if calc_time > self.__calc_time_warn_thr:
                self.__log.warning("Warning: One trajectory generation iteration took more than %.3fs (Actual " # 실행하면 나오는 주로 나오던 멘트
                                   "calculation time: %.3fs)" % (self.__calc_time_warn_thr, calc_time))
            self.__log.debug("Update frequency: %.2f Hz" % (1.0 / max(calc_time, 0.001)))

            # get moving average of calc time (smooth out some outliers)
            if len(self.__calc_buffer) >= self.__calc_time_buffer_len:
                self.__calc_buffer.pop(0)
            self.__calc_buffer.append(calc_time)
            calc_time_avg = float(np.sum(self.__calc_buffer) / len(self.__calc_buffer))

            # get index of pose on last trajectory based on the previous calculation time and vel profile
            # divide element lengths by the corresponding velocity in order to obtain a time approximation
            s_past = np.diff(self.__last_bp_action_set[action_id_sel][idx_sel_traj][1:, 0])
            v_past = self.__last_bp_action_set[action_id_sel][idx_sel_traj][1:-1, 5]
            t_approx = np.divide(s_past, v_past, out=np .full(v_past.shape[0], np.inf), where=v_past != 0)

            # force constant trajectory for a certain amount of time (here: upper bounded calculation time)
            t_const = min(calc_time_avg * self.__calc_time_safety, 0.5)

            # find cumulative time value larger than estimated calculation time
            next_idx = (np.cumsum(t_approx) <= t_const).argmin() + 1

            # Get first node after "next_idx_corr"
            last_node_idx = self.__last_action_set_node_idx[action_id_sel][idx_sel_traj]
            # print("last_node_idx: ", last_node_idx)
            node_coords = self.__last_action_set_path_param[action_id_sel][idx_sel_traj][last_node_idx, 0:2]
            predicted_pos = self.__last_bp_action_set[action_id_sel][idx_sel_traj][next_idx, 1:3]
            # predicted_pos에 가까운 노드 인덱스 찾는 과정.
            start_node_idx = graph_ltpl.helper_funcs.src.get_s_coord.get_s_coord(ref_line=node_coords,
                                                                                 pos=predicted_pos,
                                                                                 only_index=True)[1][1]

            loc_path_start_idx = self.__last_action_set_node_idx[action_id_sel][idx_sel_traj][start_node_idx]

            self.__start_node = self.__last_action_set_nodes[action_id_sel][idx_sel_traj][start_node_idx]

            # get nodes of last solution (used to reduce the cost on those segments)
            last_solution_nodes = self.__last_action_set_nodes[action_id_sel][idx_sel_traj][start_node_idx:]
```

### (else) 첫 calc_path 단계
```python
        else:
            self.__last_path_timestamp = time.time() 
            last_solution_nodes = None

            if const_path_seg_exists and self.__start_node in self.__last_action_set_nodes[action_id_sel][idx_sel_traj]:
                start_node_pos = self.__graph_base.get_node_info(layer=self.__start_node[0],
                                                                 node_number=self.__start_node[1])[0]
                loc_path_start_idx = graph_ltpl.helper_funcs.src.closest_path_index.\
                    closest_path_index(path=self.__last_action_set_path_param[action_id_sel][idx_sel_traj][:, 0:2],
                                       pos=start_node_pos)[0][0] 
                start_node_idx = self.__last_action_set_nodes[action_id_sel][idx_sel_traj].index(self.__start_node)
            else:
                loc_path_start_idx = 0
                start_node_idx = 0
```
- start node가 action_set_nodes에 있는 경우
  - start node의 pos를 get_node_info()를 통해 가져온다.
    - get_node_info()의 return값: array(pos[x,y], psi, raceline(bool), parents) 중에서 [0]이므로 [x,y]만 가져다가 쓴다.
  - `loc_path_start_idx`: closest_path_index()를 통하여 start node로부터 path에 있는 노드 중 가장 가까운 위치에 있는 노드의 index를 가져온다. <span style="color:orange">(이름이 직관적 X)</span>
  - `start_node_idx`: self.__last_action_set_nodes는 지금까지 경로로 선택해 온 노드가 저장[layer, node]되어 있다. 그래서 현재 시작하는 start_node의 index를 가져온다.


```python
        const_path_seg = None
        if const_path_seg_exists:
            # 이전 경로~시작 노드까지의 모든 샘플링된 점의 파라미터 
            const_path_seg = self.__last_action_set_path_param[action_id_sel][idx_sel_traj][:loc_path_start_idx + 1, :] 
        # 생성된 경로의 노드 리스트, 각 노드의 인덱스 등등 
        action_set_nodes, action_set_node_idx, action_set_coeff, action_set_path_param, action_set_red_len, \
            self.__closest_obj_index = (graph_ltpl.online_graph.src.main_online_path_gen.
                                        main_online_path_gen(graph_base=self.__graph_base,
                                                             start_node=self.__start_node,
                                                             obj_veh=self.__obj_veh,
                                                             obj_zone=self.__obj_zone,
                                                             last_action_id=action_id_sel,
                                                             max_solutions=self.__max_solutions,
                                                             const_path_seg=const_path_seg,
                                                             pos_est=self.__pos_est,
                                                             last_solution_nodes=last_solution_nodes,
                                                             w_last_edges=self.__w_last_edges))

```
- 이전 경로가 유효하다면
    - `const_path_seg`:이전 경로에서 start node까지의 모든 샘플링 포인트의 파라미터
  
예를 들어, 차량이 이미 달리고 있던 경로가 100개의 점으로 이루어져 있고, 30번째 점부터 새로운 경로를 이어서 붙이려고 한다면
`const_path_seg`에는 0번~30번 인덱스의 모든 경로점 정보가 저장된다.
즉, 경로의 앞부분(재사용하는 구간)이 저장된다.

**[main_online_path_gen](main_online_path_gen.md)**

### 루프 시작
새로 탐색하는 경로와 차량이 이미 주행 중인 기존 경로를 이어붙이는 역할 

```python
for action_id in action_set_nodes.keys():
            if action_set_nodes[action_id]:
                if const_path_seg_exists:
                    for i in range(len(action_set_nodes[action_id])): 
                        if loc_path_start_idx > 0:
```
- 모든 action set에 대하여 (1)action에 대한 trajectory가 있을 경우 (2) 해당 action에 대하여 차량이 이미 그 상태인 경우에 대하여 각 경로 후보마다 반복한다. 

### 확정된 경로에다가 새로 계산한 경로를 이어붙이는 작업

```python
if loc_path_start_idx > 0:
                            # path parameters
                            action_set_path_param[action_id][i] = np.concatenate((
                                self.__last_action_set_path_param[action_id_sel][idx_sel_traj][:loc_path_start_idx, :],
                                action_set_path_param[action_id][i])) # 원래 있던 path_param에 이어붙이기.
```
- `loc_path_start_idx > 0`이면, 기존 고정 경로의 앞부분(loc_path_start_idx까지)을 새로 계산한 경로 앞에 붙인다. 
  -  `self.__last_action_set_path_param[action_id_sel][idx_sel_traj][:loc_path_start_idx, :]`
: 기존 경로의 앞부분 path parameter(좌표, 헤딩, 커브, 등).
  - `action_set_path_param[action_id][i]`
    : 새로 탐색된 경로의 path parameter.
두 부분을 np.concatenate로 이어붙인다.

## element 길이 계산 보정 작업 
```python
                            # element length
                            if np.size(self.__last_action_set_path_param[action_id_sel][idx_sel_traj], axis=0) == \
                                    loc_path_start_idx:
                                j = loc_path_start_idx - 1
                                action_set_path_param[action_id][i][j, 4] = np.sqrt(
                                    np.power(np.diff(action_set_path_param[action_id][i][j:j + 2, 0]), 2)
                                    + np.power(np.diff(action_set_path_param[action_id][i][j:j + 2, 1]), 2))
```
- 만약 경로가 잘려진 부분이 기존 경로의 맨 끝이라면, 새 경로의 첫 점과의 거리를 다시 계산한다.

### 노드 인덱스 업데이트(경로 재조합한 뒤, 노드 인덱스 동기화)

```python
                        # Update node index reference (add the indexes of the nodes cut away beforehand and correct
                        # numbers according to segment added in the line above)
                        action_set_node_idx[action_id][i] = np.concatenate(
                            (np.array(self.__last_action_set_node_idx[action_id_sel][idx_sel_traj][:start_node_idx]),
                             (np.array(action_set_node_idx[action_id][i]) + loc_path_start_idx)))

                        # Update nodes and spline coefficients
                        if start_node_idx > 0:
                            # nodes
                            action_set_nodes[action_id][i] = np.concatenate(
                                (self.__last_action_set_nodes[action_id_sel][idx_sel_traj][:start_node_idx],
                                 action_set_nodes[action_id][i])).tolist()

                            # spline coefficients
                            action_set_coeff[action_id][i] = np.concatenate(
                                (self.__last_action_set_coeff[action_id_sel][idx_sel_traj][:start_node_idx],
                                 action_set_coeff[action_id][i]))
```
- 노드 인덱스(경로상의 노드 번호)도 고정경로와 새롭게 계산한 경로를 합친다.
- 새 경로의 인덱스는 기존 경로의 길이만큼 offset을 더한다.
- 노드와 spline 계수를 합친다. 
  
## 모든 action set이 비어 있는 경우
이 경우에는 새로운 경로 탐색에 실패한 것. 

```python
 # If all action sets are empty, print warning
        if not bool([a for a in action_set_nodes.values() if a != []]):
            self.__log.critical(
                "Could not find a path solution for any of the points in the given destination layer! "
                "Track useems to be blocked.")
```
- 목표 지점에 대하여 경로 탐색에 모두 실패한 경우
  - log로 목표지점에 대하여 경로를 찾지 못하였다. 트랙이 막혀 있을 수 있다는 로그 출력.

### path solution이 없으므로, 이전에 계산한 constant segment를 return하기 위한 보정 작업 

```python
            # if no path solution found, add constant path segment, if available
            if const_path_seg_exists and const_path_seg.shape[0] > 2:
                # increment loc_path_start_idx and start_node_idx, since we want to include the node in the path
                loc_path_start_idx += 1
                start_node_idx += 1

                if loc_path_start_idx > 0:
                    # path parameters
                    action_set_path_param[action_id_sel] = \
                        [self.__last_action_set_path_param[action_id_sel][idx_sel_traj][:loc_path_start_idx, :]]

                # Update node index reference (add the indexes of the nodes cut away beforehand and correct
                # numbers according to segment added in the line above)
                action_set_node_idx[action_id_sel] = \
                    [np.array(self.__last_action_set_node_idx[action_id_sel][idx_sel_traj][:start_node_idx])]

                # Update nodes and spline coefficients
                if start_node_idx > 0:
                    # nodes
                    action_set_nodes[action_id_sel] = \
                        [self.__last_action_set_nodes[action_id_sel][idx_sel_traj][:start_node_idx]]

                    # spline coefficients
                    action_set_coeff[action_id_sel] = \
                        [self.__last_action_set_coeff[action_id_sel][idx_sel_traj][:start_node_idx]]

                action_set_red_len[action_id_sel] = [True]

        # After combination of old and new calculations -> write "new" calculation to "old" memory
        self.__last_action_set_nodes = action_set_nodes
        self.__last_action_set_node_idx = action_set_node_idx
        self.__last_action_set_coeff = action_set_coeff
        self.__last_action_set_path_param = action_set_path_param
        self.__last_action_set_red_len = action_set_red_len

        # return characteristic parameters (e.g. for logging purposes)
        return self.__last_action_set_path_param, self.__start_node, self.__last_action_set_nodes, const_path_seg

```
- 고정 경로가 있고, 그 길이가 2 이상이면(즉, 최소한으로라도 이전 경로를 사용할 수 있으면) 아래 코드 실행.
  - 경로의 앞부분만 이라도 살려서 반환한다.
    - `loc_path_start_idx`, `start_node_idx`를 1씩 증가시켜, 경로 일부라도 결과값으로 포함.
    - `action_set_path_param[action_id_sel]` = ...
      - 경로 파라미터 일부만 잘라서 다시 저장.
    - `action_set_node_idx[action_id_sel]` = ...
      - 노드 인덱스도 앞부분만 저장.
    - `action_set_nodes[action_id_sel]` = ...
      - 노드 리스트도 앞부분만 저장.
    - `action_set_coeff[action_id_sel]` = ...
      - 스플라인 계수도 앞부분만 저장.
    - `action_set_red_len[action_id_sel]` = True
      - 이 경로는 축소된 경로임을 표시.

## 요약
차량이 이미 따라가던 경로가 있다면, 그 앞부분(차량이 주행한/주행할 구간)을 잘라서 새로 탐색된 경로 앞에 붙인다.
경로의 여러 정보(좌표, 노드 인덱스, 스플라인 계수 등)도 똑같이 앞부분과 뒷부분을 합친다.
이 과정을 통해 "경로의 앞부분은 보존, 뒷부분은 새로 탐색"하는 online trajectory 연결이 이루어진다.