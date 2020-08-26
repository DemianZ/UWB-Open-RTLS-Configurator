import json
from math import sqrt

import matplotlib
import matplotlib.pyplot as plt
import numpy as np

SPEED_OF_LIGHT = 299792458
DWT_TIME_UNITS = float(1.0/499.2e6/128.0)


class Anchor:
	def __init__(self, id, type, x, y, z):
		self.id = id
		self.type = type
		self.x = x
		self.y = y
		self.z = z

	type = {
		"POS": 1,
		"SYNC": 2,
	}


def get_anchor(id: str, anchor_list: list) -> Anchor:
	for i in range(len(anchor_list)):
		if anchor_list[i].id == id:
			return anchor_list[i]


def compute_shift_dwm(anchor_1: Anchor, anchor_2: Anchor) -> float:
	return sqrt((anchor_2.x - anchor_1.x) ** 2 +
				(anchor_2.y - anchor_1.y) ** 2 +
				(anchor_2.z - anchor_1.z) ** 2)


n_anchors = 4
n_sync = 1

anchors_pos = dict()
anchors_sync = dict()

anchors_pos["11"] = Anchor("11", Anchor.type["POS"],  3.795, 0.255, 1.090)
anchors_pos["12"] = Anchor("12", Anchor.type["POS"],  0.347, 0.240, 1.090)
anchors_pos["13"] = Anchor("13", Anchor.type["POS"],  0.256, 2.820, 1.115)
anchors_pos["14"] = Anchor("14", Anchor.type["POS"],  3.640, 2.330, 1.060)

anchors_sync["15"] = Anchor("15", Anchor.type["SYNC"],  1.110, 1.160, 1.090)


with open('../logs/tdoa_data.json', 'r') as tdoa_data:
	log_data = json.load(tdoa_data)

records_num = len(log_data)

for nn, record in log_data.items():
	par_list = record.replace(':', ',').split(', ')
	if "TX_TS" in record:
		log_data[nn] = {
			"type": par_list[0],
			"id": par_list[1].split("=")[1],
			"tx_id": par_list[2].split("=")[1],
			"NN": int(par_list[3].split("=")[1]),
			"TX_TS": int(par_list[4].split("=")[1]),
			"RX_TS": int(par_list[5].split("=")[1]),
			"K": 0
		}
	else:
		log_data[nn] = {
			"type": par_list[0],
			"id": par_list[1].split("=")[1],
			"tx_id": par_list[2].split("=")[1],
			"NN": int(par_list[3].split("=")[1]),
			"TS": int(par_list[4].split("=")[1]),
		}


sync_epoch = dict()			# "sync_node_id": { "anchor_id": { "TS": int(..), "NN": int(..)}

blink_epoch = dict()		# "anchor_id": { "tag_id": {"TS": int(), "NN": int()} }

collecting_sync = True
sync_collected = False
sync_cnt = 0


def flush_sync_epoch():
	global sync_cnt
	sync_cnt = 0
	for an_sync in anchors_sync:
		sync_epoch[an_sync] = dict()


def flush_blink_epoch():
	global blink_epoch
	blink_epoch = dict()


prev_record = dict()


def collect_sync_data():
	global sync_cnt, collecting_sync, sync_collected, k_koeff, prev_record, sync_epoch

	sync_id = record["tx_id"]
	anchor_id = record["id"]

	if anchor_id not in sync_epoch[sync_id]:
		sync_epoch[sync_id][anchor_id] = dict()
	if anchor_id not in prev_record:
		prev_record[anchor_id] = dict()

	k_koeff = 0

	if len(prev_record[anchor_id]):
		if int(record["NN"]) - (prev_record[anchor_id]["NN"]) != 1:
			flush_sync_epoch()
			collecting_sync = True
			sync_collected = False
			return

		drx = record["RX_TS"] - prev_record[anchor_id]["RX_TS"]
		dtx = record["TX_TS"] - prev_record[anchor_id]["TX_TS"]
		k_koeff = drx/dtx
		prev_record[anchor_id] = record
	else:
		prev_record[anchor_id] = record

	sync_epoch[sync_id][anchor_id]["TX_TS"] = record["TX_TS"]
	sync_epoch[sync_id][anchor_id]["RX_TS"] = record["RX_TS"]
	sync_epoch[sync_id][anchor_id]["NN"] = record["NN"]
	sync_epoch[sync_id][anchor_id]["K"] = k_koeff

	if len(sync_epoch[sync_id]) == n_anchors:
		collecting_sync = False
		sync_collected = True

	for epoch in sync_epoch[sync_id].values():
		if record["NN"] != epoch["NN"]:
			flush_sync_epoch()
			collecting_sync = True
			sync_collected = False


flush_sync_epoch()
flush_blink_epoch()

delta_ts = dict()


def compute_tdoa(sync_epoch: dict, blink_epoch: dict):
	deltas_seconds = dict()
	for blink_id, blink_ep in blink_epoch.items():
		if blink_id not in deltas_seconds:
			deltas_seconds[blink_id] = dict()

		for bl_an, bl_ep in blink_ep.items():
			for sync_id, sync_ep in sync_epoch.items():		# can be only one sync node
				if bl_an not in deltas_seconds[blink_id]:
					if sync_ep[bl_an]["K"] != 0:
						r_shift = compute_shift_dwm(anchors_pos[bl_an], anchors_sync[sync_id])
						tof_shift = r_shift / SPEED_OF_LIGHT  # shift in seconds
						dwm_shift = tof_shift / DWT_TIME_UNITS

						dwm_shift_total = (sync_ep[bl_an]["RX_TS"] - dwm_shift) - sync_ep[bl_an]["TX_TS"]

						dbs = bl_ep["TS"] - (sync_ep[bl_an]["RX_TS"])
						if dbs < 0:
							dbs = bl_ep["TS"] + (0xFFFFFFFFFE - (sync_ep[bl_an]["RX_TS"]))

						delta_dwm = bl_ep["TS"] - dwm_shift_total + dbs * (1 - sync_ep[bl_an]["K"])
						delta_t = delta_dwm * DWT_TIME_UNITS
						deltas_seconds[blink_id][bl_an] = {"dt": delta_t, "NN": bl_ep["NN"]}
					# else:
					# 	print(bl_ep["NN"])
	if len(deltas_seconds[blink_id]):
		return deltas_seconds


deltas_second = []


for nn, record in log_data.items():
	# Collect SYNC
	if collecting_sync is True:
		if record['type'] != "TDOA_SYNC":		# if not enough sync collected
			print('Skip epoch, waiting sync: ', record['NN'])
			flush_sync_epoch()
			prev_record = dict()
			continue

		if sync_collected is True:		# if already all syncs collected, start again
			flush_sync_epoch()
			sync_collected = False

		collect_sync_data()

	# Collect BLINK
	else:
		if record['type'] == "TDOA_SYNC":		# start new epoch
			sync_collected = False
			flush_sync_epoch()
			flush_blink_epoch()
			collect_sync_data()
			collecting_sync = True
			continue

		if record['type'] != "TDOA_BLINK":
			continue

		blink_id = record["tx_id"]
		an_id = record["id"]

		if blink_id not in blink_epoch:
			blink_epoch[blink_id] = dict()

		for epoch in blink_epoch[blink_id].values():
			if record["NN"] != epoch["NN"]:
				flush_blink_epoch()
				blink_epoch[blink_id] = dict()
				break

		if an_id not in blink_epoch[blink_id]:
			blink_epoch[blink_id][an_id] = {
				"TS": record["TS"],
				"NN": record["NN"]
			}

		for epoch in blink_epoch[blink_id].values():
			if record["NN"] != epoch["NN"]:
				flush_sync_epoch()
				flush_blink_epoch()
				collecting_sync = True
				sync_collected = False
				break

		if collecting_sync is True:
			continue
		try:
			if len(blink_epoch[blink_id]) == n_anchors:
				d_sec = compute_tdoa(sync_epoch, blink_epoch)
				if d_sec is not None:
					deltas_second.append(d_sec)
				else:
					print("Skip, no sync epoch: ", blink_epoch[blink_id][an_id]["NN"])
				flush_blink_epoch()
		except Exception as e:
			print(str(e) + str(record["NN"]))


d1 = list()
d2 = list()
d3 = list()
d4 = list()

d1d2 = list()
d1d3 = list()
d1d4 = list()

nn = list()

for delta in deltas_second:
	nn.append(int(delta["21"]["11"]["NN"]))

	d1d2.append(float(delta["21"]["11"]["dt"] - delta["21"]["14"]["dt"]) * SPEED_OF_LIGHT)
	d1d3.append(float(delta["21"]["12"]["dt"] - delta["21"]["14"]["dt"]) * SPEED_OF_LIGHT)
	d1d4.append(float(delta["21"]["13"]["dt"] - delta["21"]["14"]["dt"]) * SPEED_OF_LIGHT)

	d1.append(float(delta["21"]["11"]["dt"]))
	d2.append(float(delta["21"]["12"]["dt"]))
	d3.append(float(delta["21"]["13"]["dt"]))
	d4.append(float(delta["21"]["14"]["dt"]))

plt.figure(1)
ax1 = plt.subplot(411)
plt.plot(nn, d1)

ax2 = plt.subplot(412)
plt.plot(nn, d2)

ax3 = plt.subplot(413)
plt.plot(nn, d3)

ax4 = plt.subplot(414)
plt.plot(nn, d4)


plt.figure(2)
ax5 = plt.subplot(311)
plt.plot(nn[:100], d1d2[:100])

ax6 = plt.subplot(312)
plt.plot(nn[:100], d1d3[:100])

ax7 = plt.subplot(313)
plt.plot(nn[:100], d1d4[:100])

plt.show()
