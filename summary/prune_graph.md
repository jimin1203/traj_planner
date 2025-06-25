# prune_graph
cyclic graph 안에서 도달하지 않는 구간에 한해 node와 edge를 graph에서 제거 
더 이상 삭제할 것이 없을 때까지 반복
닫힌 그래프가 아닌 경우, 시작/마지막 레이어는 삭제하지 않도록 한다.
## INPUT
```python
def prune_graph(graph_base: graph_ltpl.data_objects.GraphBase.GraphBase,
                closed: bool = True) -> None:
```
- `graph_base`
- `closed`

```python
    j = 0
    rmv_cnt_tot = 0
    nodes = graph_base.get_nodes()
```
- `j`: 반복 횟수 
- `rmv_cnt_tot`: 삭제한 edge 개수
- `nodes`: 그래프의 모든 노드 리스트

## 반복 순회하면서 prune
```python
    while True:
        rmv_cnt = 0
        for i, node in enumerate(nodes):
            tph.progressbar.progressbar(min(j * len(nodes) + i, len(nodes) * 10 - 2),
                            len(nodes) * 10 - 1, prefix="Pruning graph    ")

            if not closed and (node[0] == graph_base.num_layers - 1 or node[0] == 0):
                continue
```
- `rmv_cnt` 이번 사이클에서 삭제한 edge 개수
- 그래프가 닫혀 있지 않으면, 시작이나 마지막(num_layers-1) 레이어의 노드는 건너 뛴다.

## 노드의 자식/부모 정보 얻기
```python
            _, _, _, children, parents = graph_base.get_node_info(layer=node[0],
                                                                  node_number=node[1],
                                                                  return_child=True,
                                                                  return_parent=True)
```
-
**[graph_base.get_node_info](graph_base.md)**

## 자식 또는 부모가 없는 노드 처리
```python
            if not children or not parents:
                # if no children or no parents, remove all connecting edges
                if not children:
                    for parent in parents:
                        rmv_cnt += 1
                        graph_base.remove_edge(start_layer=parent[0],
                                               start_node=parent[1],
                                               end_layer=node[0],
                                               end_node=node[1])
                else:
                    for child in children:
                        rmv_cnt += 1
                        graph_base.remove_edge(start_layer=node[0],
                                               start_node=node[1],
                                               end_layer=child[0],
                                               end_node=child[1])
```
- 자식이 없으면, 부모와 연결된 edge 삭제
- 부모가 없으면, 자식과 연결된 edge 삭제

**[graph_base.remove_edge](graph_base.md)**

## 더이상 삭제할 edge 없는 경우 반복 종료
```python
        if rmv_cnt == 0:
            break
        else:
            rmv_cnt_tot += rmv_cnt

        j += 1
```
- 삭제한 edge가 없는 경우, 반복 종료
- 삭제한 edge가 있는 경우, 삭제한 edge 개수 update
- 반복 cnt(`j`) 증가

## 진행상황 및 삭제한 edge 결과 print
```python
    tph.progressbar.progressbar(100, 100, prefix="Pruning graph    ")

    if rmv_cnt_tot > 0:
        print("Removed %d edges, identified as dead ends!" % rmv_cnt_tot)
```

