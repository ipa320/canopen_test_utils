#!/usr/bin/env python

PDOs={
"RPDO1": [1, ('6040',     "control_word",                 2), ('6060',     "op_mode",                      1)],
"RPDO2": [1, ('607A',     "target_position",              4), ('60FF',     "target_velocity",                 4)],
"RPDO3": [1, ('60C1sub1', "target_interpolated_position", 4)],
"RPDO4": [1, ('60FEsub1',     "digital_outputs",          4)],

"TPDO1": [1, ('6041',     "status_word",                  2), ('6061',     "op_mode_display",              1)],
"TPDO2": [1, ('6064',     "actual_position",              4)],
"TPDO3": [1, ('606C',     "actual_velocity",              4)],
"TPDO4": [1, ('60FD',     "digital_inputs",               4)],
}

if __name__ == "__main__":
    import pdo, sys

    fname = sys.argv[1]
    pdo.patch_all(fname, fname, PDOs)
