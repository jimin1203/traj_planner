# gen_edges
주어진 노드 정보(위치,방향 `state_pos`)로부터 각 노드 쌍 사이의 spline 곡선을 생성하고 그래프 구조의 edge로 추가한다. 또한, 차량의 최소 회전반경, 속도 등 조건을 만족하지 못하는 경로는 제거한다.

## INPUT
```python
def gen_edges(state_pos: np.ndarray, # 노드의 x, y, 방향 포함한 2D 배열
              graph_base: graph_ltpl.data_objects.GraphBase.GraphBase, # 그래프 정보를 저장하는 객체(data_objects폴더 안 graphbase.py 안 GraphBase class로 하는)
              stepsize_approx: float, # spline 샘플링할 때의 간격
              min_vel_race: float = 0.0, # 최소 허용 속도 비율(0~1) 
              closed: bool = True) -> None: # track이 닫힌 루프 형태인지 확인.
```
- `state_pos`: 노드의 [x,y]와 heading 정보 
- `graph_base`: 그래프 정보를 관할하는 객체
- `stepsize_approx`: 스플라인 곡선을 샘플링할 때의 간격
- `min_vel_race`: 최소 허용 속도 비율(0~1)
- `closed`: 트랙이 페곡선인지 여부
  

## 입력값 검증
```python
    if graph_base.lat_offset <= 0:
        raise ValueError('Requested to small lateral offset! A lateral offset larger than zero must be allowed!')
```
- lateral_offset(레이어 간 간격)이 0 이하면 Error

## raceline의 spline 계산
```python
    raceline_cl = np.vstack((graph_base.raceline, graph_base.raceline[0]))
    x_coeff_r, y_coeff_r, _, _ = tph.calc_splines.calc_splines(path=raceline_cl)
```
- `raceline_cl`: 닫힌 트랙(루프)라면, 레이싱라인의 시작점을 끝에도 붙여서 완전한 루프를 만든다.
  - np.vstack(A, B) = A의 마지막에 B를 한 행으로 추가 
- `calc_splines`: 경로를 따라 x, y 각각에 대한 스플라인(곡선 계수)을 계산한다.
  - 이 계수들은 나중에 스플라인 곡선을 그리고, 샘플링하고, 곡률, 방향 등을 계산할 때 사용

**[tph.calc_splines](tph.calc_splines.md)**

## 각 layer(트랙의 샘플링 위치)별로 순회
```python
    tic = time.time()

    for i in range(len(state_pos)):
        tph.progressbar.progressbar(i, len(state_pos) - 1, prefix="Calculate splines")
        start_layer = i
        end_layer = i + 1

        if end_layer >= len(state_pos):
            if closed:
                end_layer = end_layer - len(state_pos)
            else:
                break
```
- `end_layer`가 전체 레이어 수를 넘으면(마지막 레이어의 다음은 0번째로), 트랙이 루프(closed)면 0으로, 아니면 종료.

## 시작 layer에서 각 노드에 연결 가능한 end 노드 찾기
```python
        for start_n in range(len(state_pos[start_layer][0])):
            end_n_ref = graph_base.raceline_index[end_layer] + start_n - graph_base.raceline_index[start_layer]

            d_start = state_pos[start_layer][0][start_n, :]
            d_end = state_pos[end_layer][0][max(0, min(len(state_pos[end_layer][0]) - 1, end_n_ref)), :]
            dist = np.sqrt(np.power(d_end[0] - d_start[0], 2) + np.power(d_end[1] - d_start[1], 2))

            lat_steps = int(round(dist * graph_base.lat_offset / graph_base.lat_resolution))
```
- `start_n`: 시작 레이어에서의 각 노드 인덱스
- `end_n_ref`: 시작/종료 레이어에서의 raceline 기준 인덱스 차이(같은 lateral offset을 맞추기 위한 계산)
- `d_start`, `d_end`: 실제 x, y좌표
- `dist`: 두 노드 간 거리
- `lat_steps`: 거리와 lateral offset, 해상도(lat_resolution)에 따라 연결 가능한 end 노드의 범위(몇 개까지 연결할지)

## 연결되는 end 노드들에 대하여 반복
```python
            for end_n in range(max(0, end_n_ref - lat_steps),
                               min(len(state_pos[end_layer][0]), end_n_ref + lat_steps + 1)):
                if (graph_base.raceline_index[end_layer] == end_n) and \
                        (graph_base.raceline_index[start_layer] == start_n):
                    x_coeff = x_coeff_r[start_layer, :]
                    y_coeff = y_coeff_r[start_layer, :]
                else:
                    x_coeff, y_coeff, _, _ = tph.calc_splines.\
                        calc_splines(path=np.vstack((state_pos[start_layer][0][start_n, :],
                                                     state_pos[end_layer][0][end_n, :])),
                                     psi_s=state_pos[start_layer][1][start_n],
                                     psi_e=state_pos[end_layer][1][end_n])

                graph_base.add_edge(start_layer=start_layer, start_node=start_n, end_layer=end_layer,
                                    end_node=end_n,
                                    spline_coeff=[x_coeff, y_coeff])
```
- raceline 노드끼리 연결 시에는 이미 계산한 스플라인 계수를 사용한다.
- 그렇지 않으면 현재 노드~연결할 노드 쌍에 대해 스플라인 계수를 새로 계산한다.
- `add_edge`로 edge 정보를 그래프에 저장한다.

**[graph_base.add_edge](graph_base.md)**

## sampling 및 curvature/heading 계산
```python
    toc = time.time()
    print("Spline generation and edge definition took " + '%.3f' % (toc - tic) + "s")

    tic = time.time()
    rmv_cnt = 0
    edge_cnt = 0
    for i in range(graph_base.num_layers):
        tph.progressbar.progressbar(i, len(state_pos) - 1, prefix="Sampling splines ")
        start_layer = i
        for s in range(graph_base.nodes_in_layer[start_layer]):
            pos, psi, raceline, children, _ = graph_base.get_node_info(layer=start_layer,
                                                                       node_number=s,
                                                                       return_child=True)
            for node in children:
                edge_cnt += 1

                end_layer = node[0]
                e = node[1]
                spline = graph_base.get_edge(start_layer=start_layer,
                                             start_node=s,
                                             end_layer=end_layer,
                                             end_node=e)[0]
                x_coeff = np.atleast_2d(spline[0])
                y_coeff = np.atleast_2d(spline[1])

                spline_sample, inds, t_values, _ = tph.interp_splines.interp_splines(coeffs_x=x_coeff,
                                                                                     coeffs_y=y_coeff,
                                                                                     stepsize_approx=stepsize_approx,
                                                                                     incl_last_point=True)

                psi, kappa = tph.calc_head_curv_an.calc_head_curv_an(coeffs_x=x_coeff,
                                                                     coeffs_y=y_coeff,
                                                                     ind_spls=inds,
                                                                     t_spls=t_values)
```
- 각 layer, 각 노드별로 연결된 모든 child(연결된 노드)들을 순회
- 각 edge의 (x, y) spline 계수로 실제 경로를 샘플링 (interp_splines)하여 경로상의 여러 점을 얻고,
- 각 샘플점에서 방향(psi), 곡률(kappa)을 계산 (calc_head_curv_an).

**[tph.__interp_splines](tph.interp_splines.md)**
**[tph.calc_head_curv_an](tph.calc_head_curv_an.md)**

## 차량의 속도와 회전 반경 조건 체크
```python
                vel_rl = graph_base.vel_raceline[i] * min_vel_race
                min_turn = np.power(vel_rl, 2) / 10.0

                if (all(abs(kappa) <= 1 / graph_base.veh_turn) and all(abs(kappa) <= 1 / min_turn)) or \
                        (raceline and graph_base.get_node_info(layer=end_layer, node_number=e)[2]):
                    graph_base.update_edge(start_layer=start_layer,
                                           start_node=s,
                                           end_layer=end_layer,
                                           end_node=e,
                                           spline_x_y_psi_kappa=np.column_stack((spline_sample, psi, kappa)))
                else:
                    graph_base.remove_edge(start_layer=start_layer,
                                           start_node=s,
                                           end_layer=end_layer,
                                           end_node=e)
                    rmv_cnt += 1
```
- `vel_rl`(해당 구간에서 허용할 최소 속도): 해당 구간의 racing 속도 × 최소 허용 속도 비율
- `min_turn`: 주어진 속도에서 차량이 최대한 버틸 수 있는 곡률(최소 회전 반경)
- 조건 검사:
  - 곡률이 최소 회전반경 이내인지+속도 기반 회전반경 한계 이내인지를 모두 만족하면 edge를 graph에 남김(`update_edge`)
  - 그렇지 않으면 edge를 삭제(`remove_edge`)
- raceline(최적 경로)만큼은 무조건 남길 수 있도록 처리.

## 결과
```python
    toc = time.time()
    print("Spline sampling took " + '%.3f' % (toc - tic) + "s")

    print("Added %d splines to the graph!" % edge_cnt)
    if rmv_cnt > 0:
        print("Removed %d splines due to violation of the specified vehicle's turn radius or velocity aims!" % rmv_cnt)
```
- 전체 실행 시간, 남은 edge 개수, 삭제된 edge 개수(회전반경, 속도 조건 위반)를 출력