# import_globtraj_csv
graph_ltpl/imp_global_traj

## load data from csv file
```python
def import_globtraj_csv(import_path: str) -> tuple:
  
    csv_data_temp = np.loadtxt(import_path, delimiter=';')

    refline = csv_data_temp[:-1, 0:2] # 마지막 행을 제외한 모든 행, 1~2번째 열까지지

    width_right = csv_data_temp[:-1, 2]
    width_left = csv_data_temp[:-1, 3]

    normvec_normalized = csv_data_temp[:-1, 4:6]

    alpha = csv_data_temp[:-1, 6]

    # get racline segment lengths
    length_rl = np.diff(csv_data_temp[:, 7])

    # get kappa at raceline points
    kappa_rl = csv_data_temp[:-1, 9]

    # get velocity at raceline points
    vel_rl = csv_data_temp[:-1, 10]

    return refline, width_right, width_left, normvec_normalized, alpha, length_rl, vel_rl, kappa_rl

```
- global raceline을 얻기 위하여 csv의 아래와 같은 열을 추출출
  - x_ref_m;y_ref_m;width_center_m;x_normvec;y_normvec;alpha;s_raceline;vel_rl
