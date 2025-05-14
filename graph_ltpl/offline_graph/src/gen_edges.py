import numpy as np
import time

# custom modules
import graph_ltpl

# custom packages
import trajectory_planning_helpers as tph


def gen_edges(state_pos: np.ndarray, # 노드의 x, y, 방향 포함한 2D 배열
              graph_base: graph_ltpl.data_objects.GraphBase.GraphBase, # 그래프 정보를 저장하는 객체(data_objects폴더 안 graphbase.py 안 GraphBase class로 하는)
              stepsize_approx: float, # spline 샘플링할 때의 간격
              min_vel_race: float = 0.0, # 최소 허용 속도 비율(0~1) 
              closed: bool = True) -> None: # track이 닫힌 루프 형태인지 확인.
    """
    Generate edges for a given node skeleton.

    :param state_pos:           stacked list of x and y coordinates of all nodes to be considered for edge generation
    :param graph_base:          reference to the class object holding all graph relevant information
    :param stepsize_approx:     number of samples to be generated for each spline (every n meter one sample)
    :param min_vel_race:        min. race speed compared to global race line (in percent), all splines not allowing
                                this velocity will be removed --> set this value to 0.0 in order to allow all splines
    :param closed:              if true, the track is assumed to be a closed circuit

    :Authors:
        * Tim Stahl <tim.stahl@tum.de>

    :Created on:
        28.09.2018

    """

    # ------------------------------------------------------------------------------------------------------------------
    # PREPARE DATA -----------------------------------------------------------------------------------------------------
    # ------------------------------------------------------------------------------------------------------------------

    if graph_base.lat_offset <= 0: #오프셋이 0 이하이지 않도록 검사
        raise ValueError('Requested to small lateral offset! A lateral offset larger than zero must be allowed!')

    # ------------------------------------------------------------------------------------------------------------------
    # DEFINE EDGES AND SAMPLE SPLINE COEFFICIENTS ----------------------------------------------------------------------
    # ------------------------------------------------------------------------------------------------------------------
    # raceline: global planner로 최적화한 raceline
    # calculate splines for race-line
    raceline_cl = np.vstack((graph_base.raceline, graph_base.raceline[0])) # 닫힌 경로를 위해 레이싱 라인의 첫 번째 점을 마지막에 추가.
    x_coeff_r, y_coeff_r, _, _ = tph.calc_splines.calc_splines(path=raceline_cl) # 주어진 경로(raceline_cl)를 기반으로 x, y축 spline 계수 계산 

    tic = time.time()

    # loop over start_layers
    for i in range(len(state_pos)):
        tph.progressbar.progressbar(i, len(state_pos) - 1, prefix="Calculate splines")
        start_layer = i # 인접한 두 레이어 정의
        end_layer = i + 1

        # if requested end-layer exceeds number of available layers
        if end_layer >= len(state_pos):
            if closed:
                # if closed, connect to first layers
                end_layer = end_layer - len(state_pos)
            else:
                break

        # loop over nodes in start_layer
        for start_n in range(len(state_pos[start_layer][0])):
            # get end node reference (node with same offset to race line)
            end_n_ref = graph_base.raceline_index[end_layer] + start_n - graph_base.raceline_index[start_layer]

            # determine allowed lateral offset
            #  -> get distance between start node and (if possible) central (same index) goal node
            d_start = state_pos[start_layer][0][start_n, :] # d_start: start node의 x, y좌표
            d_end = state_pos[end_layer][0][max(0, min(len(state_pos[end_layer][0]) - 1, end_n_ref)), :]
            dist = np.sqrt(np.power(d_end[0] - d_start[0], 2) + np.power(d_end[1] - d_start[1], 2)) # dist: 두 노드 간 거리(end노드와 start노드)

            #  -> get number of lateral steps based on distance, lateral resolution and allowed lateral offset p. m.
            lat_steps = int(round(dist * graph_base.lat_offset / graph_base.lat_resolution)) # lateral offset 범위 내에서 연결 가능한 end 노드의 개수

            # loop over nodes in end_layer (clipped to the specified lateral offset)
            for end_n in range(max(0, end_n_ref - lat_steps), # 연결 가능한 end 노드에 대해서 반복
                               min(len(state_pos[end_layer][0]), end_n_ref + lat_steps + 1)):
                if (graph_base.raceline_index[end_layer] == end_n) and \
                        (graph_base.raceline_index[start_layer] == start_n):
                    # if race-line element -> use race-line coeffs
                    x_coeff = x_coeff_r[start_layer, :]
                    y_coeff = y_coeff_r[start_layer, :]
                else:
                    x_coeff, y_coeff, _, _ = tph.calc_splines.\
                        calc_splines(path=np.vstack((state_pos[start_layer][0][start_n, :],
                                                     state_pos[end_layer][0][end_n, :])),
                                     psi_s=state_pos[start_layer][1][start_n],
                                     psi_e=state_pos[end_layer][1][end_n])

                # add calculated edge to graph
                graph_base.add_edge(start_layer=start_layer, start_node=start_n, end_layer=end_layer,
                                    end_node=end_n,
                                    spline_coeff=[x_coeff, y_coeff])

    toc = time.time()
    print("Spline generation and edge definition took " + '%.3f' % (toc - tic) + "s")

    # ------------------------------------------------------------------------------------------------------------------
    # SAMPLE PATH EDGES (X, Y COORDINATES) -----------------------------------------------------------------------------
    # ------------------------------------------------------------------------------------------------------------------
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
            # loop over child-nodes
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

                # Extract race speed from global race line
                vel_rl = graph_base.vel_raceline[i] * min_vel_race

                # calculate min. allowed corner radius, when assuming 10m/s² lateral acceleration
                min_turn = np.power(vel_rl, 2) / 10.0

                # check if spline violates vehicle turn radius (race line is guaranteed to be among graph set)
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

    toc = time.time()
    print("Spline sampling took " + '%.3f' % (toc - tic) + "s")

    print("Added %d splines to the graph!" % edge_cnt)
    if rmv_cnt > 0:
        print("Removed %d splines due to violation of the specified vehicle's turn radius or velocity aims!" % rmv_cnt)


# testing --------------------------------------------------------------------------------------------------------------
if __name__ == "__main__":
    pass
