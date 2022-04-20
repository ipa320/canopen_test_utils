#!/usr/bin/env python
import sys, importlib
import rospy
from std_msgs.msg import Int16
from builtins import range

if len(sys.argv) < 2:
    print("please provide mapping modulde as first argument, e.g schunk_mapping or elmo_mapping");
    exit()

PDOs = importlib.import_module(sys.argv[1]).PDOs

pubs = dict()

def hex_to_signed(source):
    """Convert a string hex value to a signed hexidecimal value.

    This assumes that source is the proper length, and the sign bit
    is the first bit in the first byte of the correct length.

    hex_to_signed("F") should return -1.
    hex_to_signed("0F") should return 15.
    """
    if not isinstance(source, str):
        raise ValueError("string type required")
    if 0 == len(source):
        raise ValueError("string is empty")
    sign_bit_mask = 1 << (len(source)*4-1)
    other_bits_mask = sign_bit_mask - 1
    value = int(source, 16)
    return -(value & sign_bit_mask) | (value & other_bits_mask)

def pub(name, value):
    global pubs
    if not name in pubs:
        print("adding publisher: ",name)
        pubs[name] = rospy.Publisher(name, Int16, queue_size=1000)
    msg = Int16()
    msg.data = hex_to_signed(value)
    pubs[name].publish(msg)

def hex_reverse(data):
    return ''.join(reversed([data[i:i+2] for i in range(0, len(data), 2)]))

def decode_pdo(can_id,data, name, start_can_id):
    i = 0
    for d in PDOs[name][1:]:
        #out += [d[1],hex_reverse(data[i:i+2*d[2] ])]
        pub(name+"_"+str(can_id-start_can_id)+"_"+d[1], hex_reverse(data[i:i+2*d[2] ]))
        i +=2*d[2]


def decode_canopen(can_id,data):
    if can_id > 0x180 and can_id <= 0x180 + 127:
        return decode_pdo(can_id,data,"TPDO1", 0x180)
    elif can_id > 0x200 and can_id <= 0x200 + 127:
        return decode_pdo(can_id,data,"RPDO1", 0x200)

    elif can_id > 0x280 and can_id <= 0x280 + 127:
        return decode_pdo(can_id,data,"TPDO2", 0x280)
    elif can_id > 0x300 and can_id <= 0x300 + 127:
        return decode_pdo(can_id,data,"RPDO2", 0x300)

    elif can_id > 0x380 and can_id <= 0x380 + 127:
        return decode_pdo(can_id,data,"TPDO3", 0x380)
    elif can_id > 0x400 and can_id <= 0x400 + 127:
        return decode_pdo(can_id,data,"RPDO3", 0x400)

    elif can_id > 0x480 and can_id <= 0x480 + 127:
        return decode_pdo(can_id,data,"TPDO4", 0x480)
    elif can_id > 0x500 and can_id <= 0x500 + 127:
        return decode_pdo(can_id,data,"RPDO4", 0x500)

    return

if __name__ == "__main__":
    rospy.init_node("pub_pdo")
    start = -1

    for line in sys.stdin:
        parts = line.split()
        if start < 0:
            for i in range(len(parts)):
                if 'can' in parts[i]:
                    start = i
                    break
        if '#' in parts[start+1]:
            otherparts = parts[start+1].split("#")
            can_id = int(otherparts[0],16)
            data = otherparts[1]
        else:
            can_id = int(parts[start+1],16)
            data = ''.join(parts[start+3:])
        decode_canopen(can_id,data)
