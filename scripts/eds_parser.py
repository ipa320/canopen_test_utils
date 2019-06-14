#!/usr/bin/env python3
"""
Reads in a .eds file and parses the object dictionary entries into a json file
This lets the log parser give a name for each object dictionary index

@author zoe.snape@vecna.com
"""

import json
import argparse

parser = argparse.ArgumentParser(description='Parse object dictionary from eds file')
parser.add_argument('filename', help='eds file to parse')
eds_file = parser.parse_args().filename

obj_dict = {}

with open(eds_file, 'r') as f:
    line = f.readline()
    while line != '':
        if line.startswith('['):
            line = line.strip()
            try:
                s = line.strip('[]')
                val = int(s, 16)
                line = f.readline()
                if line.startswith('ParameterName'):
                    name = line.strip().split('=')[1]
                    obj_dict[str(s)] = name
            except ValueError:
                pass
        line = f.readline()

with open('parsed_eds_parameter_dict.json', 'w') as out:
    json.dump(obj_dict, out)
