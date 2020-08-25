import socket
import time
import binascii
import struct
import tests.RTTPrP.thirdParty_motion as thirdParty_motion
import tests.RTTPrP.RTTrP as RTTrP

UDP_IP = "127.0.0.1"
UDP_PORT = 5005


class RTTrPPacket:
	def __init__(self):
		self.intHeader = 0x4154		# Big Endian: 0x4154, Little Endian: 0x5441
		self.fltHeader = 0x4334		# Big Endian: 0x4154, Little Endian: 0x5441
		self.version = 0x0002		# Value = 0x0002, will increase if header changes
		self.pID = 0				# Packet sequence number
		self.pForm = 0  			# Value = 0x00 (Raw), 0x01 (Protobuf), 0x02 (Thrift)
		self.pktSize = 0			# Size of packet in bytes including header
		self.context = 0			# User definable
		self.numMods = 0
		self.data = 0


class PacketModule:
	def __init__(self):
		self.data = None

		self.module_type = {
			"Trackable": 0x01,
			"TrackableWithTS": 0x51,
			"CentroidPosition": 0x02,
			"QuatMod": 0x03,
			"EulerMod": 0x04,
			"PointMod": 0x06,
			"CentroidAccVelMod": 0x20,
			"PointAccVelMod": 0x21
		}


class TrackableModule:
	def __init__(self):
		self.type = 0x01 	# uint8_t
		self.size = 0		# uint16_t	Size of packet module including type and size
		self.nameLen = 0 	# uint8_t	Length of Trackable Name
		self.name = ""		# Var		Name
		self.numMods = 0 	# uint8_t	Total number of sub-modules
		self.data = 0


class CentroidPosition:
	def __init__(self):
		self.type = 0x02  	# uint8_t
		self.size = 29  	# uint16_t
		self.latency = 0 	# uint16_t
		self.x = 0			# double
		self.y = 0			# double
		self.z = 0			# double
		self.data = 0


def build_centroid_position():
	submod = CentroidPosition()
	submod.latency = 100
	submod.x = 1.24
	submod.y = 4.37
	submod.z = 0.16
	submod.data = struct.pack('!BHHddd',
							  submod.type,
							  submod.size,
							  submod.latency,
							  submod.x,
							  submod.y,
							  submod.z)

	mod = TrackableModule()
	mod.name = bytes('tag_1', 'utf-8')
	mod.nameLen = len(mod.name)
	mod.numMods = 1
	mod.size = 4 + mod.nameLen + len(submod.data)
	mod.data = struct.pack("!BHB%dsB" % mod.nameLen, mod.type, mod.size, mod.nameLen, mod.name, mod.numMods)

	pack = RTTrPPacket()
	pack.pID = 1
	pack.numMods = 1
	pack.pktSize = 18 + len(mod.data) + len(submod.data)
	pack.data = struct.pack('!HHHLBHLB',
							pack.intHeader,
							pack.fltHeader,
							pack.version,
							pack.pID,
							pack.pForm,
							pack.pktSize,
							pack.context,
							pack.numMods)

	pack.data += mod.data + submod.data

	return pack.data


if __name__ == '__main__':
	sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

	while True:
		data = build_centroid_position()
		sock.sendto(data, (UDP_IP, UDP_PORT))
		time.sleep(5)
