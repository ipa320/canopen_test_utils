#!/usr/bin/env python2

import sys
from collections import defaultdict, deque
import traceback


def hex_with_len(val, l):
    return '{0:0{1}x}'.format(val,l)

def unsigned(s, l, decode):
    if decode:
        return int(s,16)
    return hex_with_len(s,l)

def signed(s, l, decode):
    w = l*4
    if decode:
        v = int(s,16)
        if v > (1 << (w-1))-1:
            v -= (1 << (w))
        return v
    else:
        if s < 0:
            s += (1 << (w))
        return hex_with_len(s,l)

def thresh(t):
    return lambda a,b: abs(a-b) > t


last = defaultdict(dict)

formats = defaultdict(lambda:(signed, thresh(10000)),mapping={
    'status_word': (unsigned, thresh(0)),
    'op_mode_display': (unsigned, thresh(0)),
})

context = deque(maxlen=20)
dump_mode = 0

def dump_context(p, context, mode):
    if mode is 1:
        for c in context:
            print c
    elif mode is 2:
        s = p[-1][0]
        test = frozenset(s[i:i + 2] for i in range(0, len(s), 2))
        f = 0
        for c in context:
            s = c[-1][0]
            for i in range(0, len(s), 2):
                if s[i:i + 2] in test:
                    if f is 0:
                        f = 1
                    else:
                        print '**',
                        break
            print c
    else:
        print "Error: dump mode not set!"
    print
while True:
    try:
        line = sys.stdin.readline()
        if not line:
            break
        p = eval(line)
        if 'TPDO' in line:
            if dump_mode is 0:
                dump_mode = 2 if isinstance(p[-1],tuple) else 1
            i = int(p[3])
            d = p[4:]
            while len(d) > 1:
                k,v,d = d[0], d[1], d[2:]
                fmt, jump = formats[k]
                dump=False
                val = fmt(v, len(v), True)
                if k in last[i]:
                    old = last[i][k]
                    if jump(val,last[i][k]):
                        dump=True
                        print 't: {}, i: {}, {} = 0x{} <> 0x{}, {} <> {}'.format(p[0],i, k, v, fmt(old, len(v), False) ,val,old)
                        del last[i][k]
                if dump:
                    dump_context(p, context, dump_mode)
                else:
                    last[i][k]=val

        context.append((p))
    except:
        print line,
        traceback.print_exc()
        break
