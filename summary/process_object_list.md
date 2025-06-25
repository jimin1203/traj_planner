## process_object_list()
data_objects/ObjectListinterface class

```python
    def process_object_list(self,
                            object_list: list) -> list:
```

```python
        if object_list is not None:
            self.__last_timestamp = time.time()

            new_vehicle_objects = []

            for object_el in object_list:
                if object_el['type'] in KNOWN_OBJ_TYPES:
                    if object_el['type'] == "physical":
                        on_track = True
                        if self.__bound1 is not None and self.__bound2 is not None:
                            on_track = graph_ltpl.online_graph.src.check_inside_bounds.\
                                check_inside_bounds(bound1=self.__bound1,
                                                    bound2=self.__bound2,
                                                    pos=[object_el['X'], object_el['Y']]) # 차량이 내부에 있는지 확인

                        if on_track:
                            if 'prediction' in object_el.keys():
                                pred = object_el['prediction']

                            else:
                                dt = 0.2  # 200ms prediction horizon
                                pred = np.zeros((1, 2))
                                pred[0, 0] = object_el['X'] - np.sin(object_el['theta']) * object_el['v'] * dt
                                pred[0, 1] = object_el['Y'] + np.cos(object_el['theta']) * object_el['v'] * dt
                                
                            veh_obj = VehObject(id_in=object_el['id'],
                                                pos_in=[object_el['X'], object_el['Y']],
                                                psi_in=object_el['theta'],
                                                radius_in=(object_el['length'] / 2.0),
                                                vel_in=object_el['v'],
                                                prediction_in=pred)
                            new_vehicle_objects.append(veh_obj)

                else:
                    self.__log.warning("Found non-supported object of type '%s' in object list!" % object_el['type'])

            self.__object_vehicles = new_vehicle_objects

        else:
            if time.time() - self.__last_timestamp > TIME_WARNING:
                if self.__last_timestamp == 0.0:
                    time_str = "so far"
                else:
                    time_str = "in the last %.2fs" % (time.time() - self.__last_timestamp)

                self.__log.warning("Did not receive an object list " + time_str + "! Check coms!")

        return self.__object_vehicles

```