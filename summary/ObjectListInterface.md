# ObjectListInterface class 

## 생성자
```
def __init__(self) -> None:

        self.__log = logging.getLogger("local_trajectory_logger")

        self.__object_vehicles = []
        self.__object_zones = []

        self.__refline = None
        self.__normvec_normalized = None
        self.__w_left = None
        self.__w_right = None
        self.__bound1 = None
        self.__bound2 = None

        self.__last_timestamp = 0.0
```

## set_track_data()
```
    def set_track_data(self,
                       refline: np.ndarray,
                       normvec_normalized: np.ndarray,
                       w_left: np.ndarray,
                       w_right: np.ndarray) -> None:

        self.__refline = refline
        self.__normvec_normalized = normvec_normalized
        self.__w_left = w_left
        self.__w_right = w_right

        self.__bound1 = refline + normvec_normalized * np.expand_dims(w_right, 1)
        self.__bound2 = refline - normvec_normalized * np.expand_dims(w_left, 1)
```
- 차량 주행 경로의 경계 정보(self.__bound1, self.__bound2)를 설정
  - bound1 = 트랙 오른쪽 경계(점 집합)
  - bound2 = 트랙 왼쪽 경계(점 집합)