#!/usr/bin/env python

import sys
from collections import deque, defaultdict
import traceback
import time
known_nodes=set()
known_data=set()
current_data = defaultdict(set)

sync_maxlen = 1000

sync_data = deque(maxlen=sync_maxlen)
last_sync = (None,None)
last_print_clock = time.clock()
last_print_t = None
last_print_counter = sync_maxlen
force_print = False

def printSyncStats(sync_data, diff):
    milliseconds =  1e-3
    mn = min(sync_data) / milliseconds
    mx = max(sync_data) / milliseconds
    avg = sum(sync_data) / len(sync_data)
    hz = 1.0/avg if avg != 0 else 0
    print("({}) {} Hz [ {} ms <= {} ms <= {} ms ] {} ms".format(time.time(), hz, mn, avg / milliseconds, mx, diff / milliseconds))
    print

while True:
    try:
        line = sys.stdin.readline()
        if not line:
            break
        now = time.clock()
        last_t, last_clock = last_sync
        p = eval(line)
        t = float(eval(p[0]))
        if last_print_t is None:
            last_print_t = t
        if 'SYNC' in line:
            if last_t is not None:
                sync_data.append(t-last_t)
                missing_nodes = known_nodes - set(current_data.iterkeys())
                if len(missing_nodes) > 0:
                    print("!!!! ({}) @ {:.6f} missing data from {}".format(now, t, missing_nodes))
                #for k,v in current_data.items():
                #    missig_data = known_data - v
                #    if len(missig_data) > 0:
                #        print "!!!! ({}) @ {:.6f} missing entries {} from {}".format(now, t, missig_data, k)
                current_data.clear()
                known_data.clear()

            last_sync = (t, now)
            last_print_counter -= 1
        if 'PDO' in line:
            i = int(p[3])
            d = p[4:]
            while len(d) > 1:
                k,v,d = d[0], d[1], d[2:]
                if k in current_data[i] and len(sync_data) > 0:
                    print("!!!! ({}) @ {:.6f} doubled data '{}' from {}".format(now, t, k, i))
                    force_print = True
                current_data[i].add(k)
                known_data.add(k)
            known_nodes.add(i)
        if len(sync_data) > 0 and ((now - last_print_clock) > 10 or (t is not None and (t - last_print_t) > 10) or force_print):
            printSyncStats(sync_data, last_clock - now)
            last_print_clock = now
            last_print_t = t
            last_print_counter = sync_maxlen
            force_print = False

    except:
        print(line,)
        traceback.print_exc()
        exit(1)

printSyncStats(sync_data, last_sync[1] - time.clock())

