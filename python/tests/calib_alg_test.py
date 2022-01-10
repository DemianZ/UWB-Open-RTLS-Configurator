import json
import numpy as np


def compute(measures, distance):
    dwt_time_units = (1.0 / 499.2e6 / 128.0)
    sof = 299702547

    iterations = 100
    candidates_num = 1000

    # Remove invalid values from measures
    for values in measures.values():
        for val in values.values():
            medium = np.median(val)
            medium_p = 1.2 * medium
            medium_m = 0.8 * medium
            for v in val:
                if (v < medium_m) or (v > medium_p):
                    val.remove(v)
    # print(measures)

    # Fill EDM matrices
    n = len(measures)
    edm_meas = np.zeros((n, n))
    edm_act = np.zeros((n, n))
    for i, (di_k, di_v) in enumerate(measures.items()):
        for j, (dj, ranges) in enumerate(di_v.items()):
            if j >= i:
                edm_meas[i, j + 1] = np.median(ranges)
                edm_act[i, j + 1] = distance
            else:
                edm_meas[i, j] = np.median(ranges)
                edm_act[i, j] = distance

    # print(edm_act)
    # print(edm_meas)

    # EDM to TOF
    tof_meas = edm_meas / sof / dwt_time_units
    tof_act = edm_act / sof / dwt_time_units

    # print('\n', tof_meas)
    # print('\n', tof_act)

    init_delay = 513
    perturb_limit = 0.2
    cand_list = list()
    cand_norm_diff = dict()

    for i in range(iterations):
        # Populate set of candidate
        if i == 0:
            # Generate a set of random delays uniformly distributed round initial delay +-6ns
            cand_list = np.random.uniform(init_delay - 6, init_delay + 6, candidates_num)
        else:
            new_cand_list = cand_list[:iterations // 4]
            for m in range(3):
                add_cand_list = np.random.uniform(-perturb_limit, +perturb_limit, iterations // 4)
                for j, cand in enumerate(new_cand_list):
                    new_cand_list += cand + add_cand_list[j]
            if i % 20 == 0:
                perturb_limit /= 2
            cand_list = new_cand_list

        # Evaluate the quality of the candidates
        # print(cand_list, '---\n', tof_meas, '---\n', )
        for cand in cand_list:
            tof_cand = tof_meas + cand
            np.fill_diagonal(tof_cand, 0)
            cand_norm_diff[cand] = np.linalg.norm(tof_act - tof_cand)

        # Sort by value - lowest error first
        cand_norm_diff = {k: v for k, v in sorted(cand_norm_diff.items(), key=lambda item: item[1])}
        cand_list = list(cand_norm_diff.keys())

    best_delay = cand_list[0]
    delay_new = (tof_meas - tof_act) / 2 + best_delay
    np.fill_diagonal(delay_new, 0)
    delays = measures
    res_delays = dict()
    for i, (node, val) in enumerate(delays.items()):
        res_delays[node] = (np.sum(delay_new, axis=0)[i] + np.sum(delay_new, axis=1))[i] / 2
    # print(res_delays)
    for node, delay in res_delays.items():
        print('{}: tx_delay - {}, rx_delay - {}'.format(node, int(delay*0.44), int(delay*0.56)))
    return res_delays


if __name__ == "__main__":
    # execute only if run as a script

    with open("../logs/calib_data.json", "r") as read_file:
        data = json.load(read_file)

    compute(data, 1.5)
