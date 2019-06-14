#!/usr/bin/env python3
"""
Parses a socketcan_interface socketcan_dump log or stdin stream
and adds human-readable comments about what the can frames mean

To parse a log, you must first dump socketcan output to a log file by running:
    $ rosrun socketcan_interface socketcan_dump can0 > socketcan.log
and then doing whatever can actions you want to log (ex running a rosnode or sending can frames)
Then:
    $ ./socketcan_dump_parser.py path/to/socketcan.log
will print an annotated version of your log file to stdout

To parse from stdin, simply pipe the output of socketcan_interface into this program:
    $ rosrun socketcan_interface socketcan_dump can0 | ./socketcan_dump_parser.py
It should print out annotated frames as they're received

If you want to use a .eds or .dcf file to annotate specific parameters, you should first use the associated
eds_parser.py script to parse that file into a json dictionary, then run this script with that dictionary as an argument

References:
  - Intro to CANopen: https://en.wikipedia.org/wiki/CANopen
  - Elmo CANopen DSP 402 manual: http://www.elmomc.com/support/manuals/MAN-CAN402IG.pdf
  - socketcan_interface package: http://wiki.ros.org/socketcan_interface

@author zoe.snape@vecna.com
"""

import json
import argparse
import sys

'''
Set of reference dictionaries
'''
# COB-IDs aka CAN message IDs
cob_ids = {
    '60': 'Receive SDO',
    '58': 'Transmit SDO',
    '70': 'heartbeat',
    '00': 'NMT message',
    '80': 'Sync',
    '18': 'Transmit PDO'}

# Network management messages
nmt_msgs = {
    '01': 'operational',
    '02': 'stopped',
    '80': 'pre-operational',
    '81': 'reset node',
    '82': 'reset communication'}

# Heartbeat messages
hb_msgs = {
    '00': 'boot up',
    '04': 'stopped',
    '05': 'operational',
    '7F': 'pre-operational'}

# Client command specifier bitmap (computer -> can device)
ccs_msgs = {
    0: 'segment download (req)',
    1: 'init download (write) (req)',
    2: 'init upload (read) (req)',
    3: 'segment upload (req)',
    4: 'aborting SDO transfer',
    5: 'SDO block uplaod',
    6: 'SDO block download'}

# Server command specifier bitmap (can device -> computer)
scs_msgs = {
    0: 'segment upload (resp)',
    1: 'segment download (resp)',
    2: 'init upload (read) (resp)',
    3: 'init download (write) (resp)',
    4: 'aborting SDO transfer',
    5: 'SDO block uplaod',
    6: 'SDO block download'}

# Translations for common states in the low byte of the statusword (6041)
statusword = {
    '50': 'switch on disabled',
    '31': 'ready to switch on',
    '33': 'switch on',
    '37': 'operation enabled',
    '18': 'fault',
    '1d': 'fault'}

# Translations for common states in the high byte of the statusword (6041)
statusword_high = {
    '02': 'target not reached',
    '12': 'mode target reached, gen target not reached',
    '06': 'target reached',
    '16': 'mode target reach and gen target reached'}

# Translations for common messages in the low byte of the controlword (6040)
controlword = {
    '06': 'enable voltage',
    '07': 'switch on',
    '0f': 'enable operation',
    '1f': 'start operation'}

# Control modes (6061)
modes_of_op = {
    '00': 'not a valid mode',
    '01': 'profiled position',
    '02': 'velocity (not supported on elmo)',
    '03': 'profiled velocity',
    '04': 'profiled torque',
    '06': 'homing',
    '07': 'interpolated position'}


def get_binary(hex_string):
    """
    Convert a hex string to a binary string
        @param hex_string: string of a hex value (e.g. '0xd')
        @returns (string): binary string of the hex value (e.g. '1101')
    """
    return '{0:04b}'.format(int(hex_string, 16))


def translate(key, dictionary):
    """
    Get an element's translation from one of the maps
        @param key: (string) key to search for
        @param dictionary: dict to search in
        @returns (string): dict[key] if key is in dict, otherwise key
    """
    return dictionary[key] if key in dictionary.keys() else key


def print_error(err_msg):
    """
    Print an eye-catching error message
        @param err_msg: string to print inside the lines of stars
    """
    print("""\n
***************************************************************
******************* ERROR : %s **********************
***************************************************************""" % err_msg)


def main():
    """
    Parse the log file
    """
    parser = argparse.ArgumentParser(description='Parse socketcan_interface socketcan_dump output')
    parser.add_argument('filename', nargs='?', help='log file to parse')
    parser.add_argument('-e', '--edsfile', default='parsed_eds_parameter_dict.json',
                        help='parsed eds file to use for mappings')
    args = parser.parse_args()
    logfile = args.filename
    eds_file = args.edsfile

    # Load object dictionary entries and parameter names parsed from a .eds or .dcf file
    parameter_name_dict = None
    if eds_file is not None:
        try:
            parameter_name_dict = json.load(open(eds_file, 'r'))
        except FileNotFoundError:
            print('Parsing without command names\n')

    if logfile is not None:
        in_stream = open(logfile, 'r')
        print('Reading from log file: %s\n' % logfile)
    else:
        in_stream = sys.stdin
        print('Reading from stdin\n')

    line_num = 1
    for line in in_stream:
        # skip any lines that aren't frames
        if not in_stream.readline().startswith('s'):
            continue

        # print line number, original frame (includes a newline) and line number again
        print(line_num, end=' ')
        print(line, end='')
        print(line_num, end=' ')
        line_num += 1

        # parse socketcan_dump format
        frame = [x.split("\t") for x in line.strip("\n").split(" ") if x != ""]
        frame = [item.zfill(2) for sublist in frame for item in sublist]
        cob_id = frame[1]
        cob_name = cob_id[:2]

        # prime cob name and id
        print(translate(cob_name, cob_ids) + " -", end=' ')

        if cob_id == '00':
            # handle NMT messages
            nmt_msg = frame[3]
            print(translate(nmt_msg, nmt_msgs), end=' ')
            print("to node #" + frame[4] + " -", end=' ')

        elif cob_id.startswith('70'):
            # handle heartbeat messages
            hb_msg = frame[3]
            print(translate(hb_msg, hb_msgs) + " from node #" + cob_id[2] + " -", end=' ')

        elif cob_id.startswith('60') or cob_id.startswith('58'):
            # parse SDO specifier bytes
            spec = frame[3]
            # command specifier bits
            cs_bits = int(get_binary(spec[0])[:3], 2)
            if cob_id.startswith('60'):
                cs_name = translate(cs_bits, ccs_msgs)
            else:
                cs_name = translate(cs_bits, scs_msgs)
            print(cs_name + " (" + str(cs_bits) + ") -", end=' ')

            if cs_bits == 4:
                print_error("SDO Aborted")

            # print data size
            data_ind_byte = get_binary(spec[1])
            num_data_bytes = 4
            if data_ind_byte[2] == '1' and data_ind_byte[3] == '1':
                num_data_bytes = 4 - (int(data_ind_byte[:2], 2))
                print(str(num_data_bytes) + " bytes of data -", end=' ')

            # parse specific object dictionary commands
            index = frame[5] + frame[4]
            # if parameter names are defined, look up command name and print data
            if parameter_name_dict is not None:
                print("command name: " + parameter_name_dict[index.upper()]
                      + " (index #" + index.upper() + "sub" + frame[6] + ") -", end=' ')
            else:
                print("command index: " + index.upper() + "sub" + frame[6] + " -", end=' ')
            data = frame[7:(7 + num_data_bytes)]
            print("with data " + "".join(data) + " -", end=' ')

            if index == '6041' and num_data_bytes == 2:
                # parse status word
                print("status: " + translate(data[0], statusword) + " - ", end=' ')
                print(translate(data[1], statusword_high) + " -", end=' ')

            elif index == '6040' and num_data_bytes == 2:
                # parse control word
                print("control command: " + translate(data[0], controlword) + " -", end=' ')
                if data[1] == '01':
                    print("halt -", end=' ')

            elif (index == '6060' or index == '6061') and num_data_bytes == 1:
                # parse mode of operation command/display
                print("mode: " + translate(data[0], modes_of_op) + " -", end=' ')
        print('\n')

    if in_stream is not sys.stdin:
        in_stream.close()


if __name__ == '__main__':
    main()
