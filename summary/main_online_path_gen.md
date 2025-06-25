## main_online_path_gen

### INPUT
```python
def main_online_path_gen(graph_base: graph_ltpl.data_objects.GraphBase.GraphBase,
                         start_node: tuple,
                         obj_veh: list,
                         obj_zone: list,
                         action_sets=True,
                         last_action_id: str = None,
                         max_solutions=1,
                         const_path_seg: np.ndarray = None,
                         pos_est: np.ndarray = None,
                         last_solution_nodes: list = None,
                         w_last_edges: list = ()) -> tuple:
```

### 
```python
    end_layer, closest_obj_index, closest_obj_node = graph_ltpl.online_graph.src.gen_local_node_template.\
        gen_local_node_template(graph_base=graph_base,
                                start_node=start_node,
                                obj_veh=obj_veh,
                                obj_zone=obj_zone,
                                last_solution_nodes=last_solution_nodes,
                                w_last_edges=w_last_edges)
```