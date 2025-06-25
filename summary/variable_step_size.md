# variable_step_size
graph_ltpl/imp_global_traj

경로를 따라 일정 간격으로 지점을 추출하되, 곡선에서는 더 촘촘하게, 직선에서는 더 넓게 지점을 선택 
- 반환값: 선택된 인덱스의 리스트
## input
| 파라미터 이름   | 타입         | 설명                                                                                  |
|----------------|--------------|--------------------------------------------------------------------------------------|
| `kappa`        | `np.ndarray` | 각 지점의 곡률(커브의 강도, 1/반지름) 값 배열                                         |
| `dist`         | `np.ndarray` | 각 지점 간의 거리(유클리드 거리 또는 스플라인 거리) 배열                              |
| `d_curve`      | `float`      | 곡선 구간에서의 최소 간격(거리, m 단위) - .init                                  |
| `d_straight`   | `float`      | 직선 구간에서의 최소 간격(거리, m 단위) - .init                                         |
| `curve_th`     | `float`      | 곡률 임계값(이 값보다 크면 곡선, 작으면 직선으로 간주) - .init                            |
| `force_last`   | `bool`       | 마지막 인덱스를 무조건 포함할지 여부 (기본값: `False`)                                 |

```python
def variable_step_size(kappa: np.ndarray,
                       dist: np.ndarray,
                       d_curve: float,
                       d_straight: float,
                       curve_th: float,
                       force_last: bool = False) -> list:
   
    next_dist = 0 # 다음 포인트 찍을 누적 거리 
    next_dist_min = 0 # 곡선구간에서 최소 간격 보장용 하한선 
    cur_dist = 0 # 지금까지 이동한 누적 거리 
    idx_array = []
    for idx, (kappa_val, dist_val) in enumerate(zip(kappa, dist)):
        if (cur_dist + dist_val) > next_dist_min:
            if abs(kappa[idx]) > curve_th: # 곡률이 임계값보다 크면 
                next_dist = cur_dist # next_dist를 현재 distance로 설정 
```
- 누적 거리가 `next_dist_min`을 넘으면(첫번째 if) 곡선 구간에서만(두번째 if) `next_dist`를 현재 위치로 당긴다.  
  - 곡선 간격이니까 여기서부터 곡선 간격으로 샘플링 시작. 
  - 그래서 현 index에서 포인트를 찍기 위해 next_dist = cur_dist를 진행함. 이러면 다음 if문에서 무조건 `idx_array`에 idx를 추가하게 된다. 
```python
        if (cur_dist + dist_val) > next_dist: # 다음 샘플링 지점에 도달했는지 
            idx_array.append(idx)

            if abs(kappa[idx]) < curve_th: # 직선 구간으로 간주
                next_dist += d_straight 
            else: # 곡선 구간으로 간주
                next_dist += d_curve
            next_dist_min = cur_dist + d_curve # 곡선 간격만큼 더한 값으로 update 

        cur_dist += dist_val # 현재까지 이동한 총 거리를 누적 
```
- 누적 거리가 `next_dist`를 넘으면 포인트(`idx_array`) 추가, `next_dist`, `next_dist_min` 업데이트 
- 현재까지 누적거리 + 이번 구간의 길이가 다음 포인트를 찍기로한 목표 거리를 넘어섰을 때 
```python
    if force_last and len(kappa) - 1 not in idx_array: # 마지막 인덱스 강제 추가 
        idx_array.append(len(kappa) - 1)

    return idx_array
```
- main_offline_callback에서 호출 시(offline.ini에 저장되어 있는 값)
  - `dist` = s(arc length)로 구간별 s가 아닌 누적된 s를 인자로 한다.
  - `d_curve` = 10.0
  - `d_straight` = 30.0
  - `curve_th` = 0.08 
- <span style="color:orange">kappa를 어떤 식으로 사용하고 있는 것인가?</span>