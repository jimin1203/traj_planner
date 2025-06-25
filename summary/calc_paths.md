## GRAPH_LTPL/calc_paths

main_min_example.py에서 **ltpl_obj.calc_paths(prev_action_id=sel_action, object_list=[])**로 호출되었다.
이때 sel_action = straight(추월하거나 따라갈 다른 차량이 없으므로), object_list=[]으로 고정.

### Input
```python
    def calc_paths(self,
                   prev_action_id: str,
                   prev_traj_idx: int = 0,
                   object_list: list = None,
                   blocked_zones: dict = None) -> dict:
```
- `prev_action_id` 이전 iteration에서 선택된 action_id (straight, left, right 등)
- `prev_traj_idx` 이전 iteration에서 선택된 경로 인덱스
  - trajectory set 내의 여러 경로 중 몇 번째 trajectory인지.
- `object_list` 각 객체를 담은 list 구조
  - 다음과 같은 key를 들고 있어야 한다.
    - type
    - x, y
    - theta ...
- `blocked zones` 주행 불가 영역에 대한 정보를 담은 dict
  - key는 Zone ID, value는 zone 정보를 담은 list
  - [blocked layer numbers, blocked node numbers, left bound of region, right bound of region]

### 입력받은 파리미터 업데이트

```python
        self.__prev_action_id = prev_action_id
        self.__prev_traj_idx = prev_traj_idx
        # print("prev_traj_idx: ", self.__prev_traj_idx) # 계속 0
```

```python
        # update internal object handles(트랙 외부 거르고 예측까지 포함된 list 리턴)
        self.__obj_veh = self.__obj_list_handler.process_object_list(object_list=object_list) 
```
**[process_object_list](process_object_list.md)** 인데 main_min_example에서는 self.__obj_veh = []이다. (다른 차량이 없어서.)

```python
        # update zones
        if blocked_zones is not None:
            for blocked_zone_id in blocked_zones.keys():
                self.__obj_zone = self.__obj_list_handler.update_zone(zone_id=blocked_zone_id,
                                                                      zone_data=blocked_zones[blocked_zone_id],
                                                                      zone_type='nodes')
        # update (clear and set new) obstacles in the scene
        self.__oth.update_objects(obj_veh=self.__obj_veh,
                                  obj_zone=self.__obj_zone)

```
- update_objects 함수는 OnlineTrajectoryHandler 객체에 obj_veh와 obj zone을 저장하고, closest_obj_index를 초기화하는 역할을 함.

```python
        # trigger local trajectory generation
        path_dict, self.__plan_start_node, self.__node_list, self.__const_path_seg = \
            self.__oth.calc_paths(action_id_sel=self.__prev_action_id,
                                  idx_sel_traj=self.__prev_traj_idx)
```
[oth.calc_paths](oth.calc_paths.md)

```python
        return path_dict
```