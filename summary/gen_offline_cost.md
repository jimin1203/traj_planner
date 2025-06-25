# gen_offline_cost
주어진 graph_base의 모든 edge에 대하여 offline cost를 부여하여 다시 해당 값을 graph에 저장한다.

## INPUT
```python
def gen_offline_cost(graph_base: graph_ltpl.data_objects.GraphBase.GraphBase,
                     cost_config_path: str):
```
- `graph_base`
- `const_config_path`: cost 계산에 쓰일 offline.ini 경로
```ini
[COST]
# -- COST --------------------------------------------------------------------------------------------------------------
# cost per meter (length) and per lateral meter added to nodes apart from raceline
w_raceline=1.0

# saturation of race line cost /m (race line cost is increased with rising lateral offset until this value is reached)
w_raceline_sat=1.0

# cost per spline meter
w_length=0.0

# cost weighting of average curvature per meter
w_curv_avg=7500.0

# cost weighting of peak curvature in the spline of interest per meter
w_curv_peak=2500.0

# virtual goal - per meter lateral offset to race line node
w_virt_goal=10000.0
```

## 데이터 준비
```python
    cost_config = configparser.ConfigParser()
    if not cost_config.read(cost_config_path):
        raise ValueError('Specified graph config file does not exist or is empty!')
    if 'COST' not in cost_config:
        raise ValueError('Specified graph config file does not hold the expected data!')
```
- configparser()를 이용해 offline.ini 읽을 준비

## 모든 엣지를 순회하며 offline cost 계산
```python
    tic = time.time()
    edges = graph_base.get_edges()
    i = 0
    for edge in edges:
        tph.progressbar.progressbar(i, len(edges) - 1, prefix="Generating cost  ")
```

**[graph_base.get_edges](graph_base.md)**

## edge 데이터 추출
```python
    spline_coeff, spline_param, offline_cost, spline_len = graph_base.get_edge(edge[0], edge[1], edge[2], edge[3])
```
- 각 엣지의 [시작 layer, 해당 node의 number, 마지막 layer, ㅎ당 node의 number]를 바탕으로 spline 계수, param, 기존 offline cost, spline 길이 가져온다.
  
**[graph_base.get_edge](graph_base.md)**

```python
        offline_cost = 0.0
```
## 평균 curvature 
``` python
        offline_cost += cost_config.getfloat('COST', 'w_curv_avg') * np.power(
            sum(abs(spline_param[:, 3])) / float(len(spline_param[:, 3])), 2) * spline_len
```
- `w_curv_avg`: 7500.0(default)
- 가중치 x 각 경로의 curvature의 절대값 평균의 제곱 x 경로 길이 
- 평균 curvature가 클수록, cost가 커진다.

## peak curvature
```python
        offline_cost += cost_config.getfloat('COST', 'w_curv_peak') * np.power(
            abs(max(spline_param[:, 3]) - min(spline_param[:, 3])), 2) * spline_len
```
- `w_curv_peak`: 2500.0(default)
- 가중치 x curvature의 최댓값과 최소값의 차이의 절대값의 제곱 x 경로 길이
- 경로 내 곡률 변화폭(peak-to-peak)이 클수록 cost가 커진다.

## path length
```python
        offline_cost += cost_config.getfloat('COST', 'w_length') * spline_len
```
- `w_length`: 0.0(default)
- 가중치 x 경로의 길이 

## raceline cost
```python
        raceline_dist = abs(graph_base.raceline_index[edge[2]] - edge[3]) * graph_base.lat_resolution
        offline_cost += min(cost_config.getfloat('COST', 'w_raceline') * spline_len * raceline_dist,
                            cost_config.getfloat('COST', 'w_raceline_sat') * spline_len)
```
  - `w_raceline`: 1.0(default)
  - `w_raceline_sat`: 1.0(default)
  - 해당 edge가 raceline에서 얼마나 떨어져 있는지
  - raceline_dist x 경로 길이 x 가중치 와 경로 길이 x saturation? 중 작은 것으로 cost 측정
    - raceline에서 멀어질수록 cost가 커지지만, 일정 거리 이상 멀어지는 경우 sat 값으로 제한한다. 

## 계산 결과 저장
```python
        graph_base.update_edge(start_layer=edge[0],
                               start_node=edge[1],
                               end_layer=edge[2],
                               end_node=edge[3],
                               offline_cost=offline_cost)
```

**[graph_base.update_edge](graph_base.md)**