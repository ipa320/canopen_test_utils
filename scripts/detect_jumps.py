#!/usr/bin/env python

import sys
from collections import defaultdict, deque
import traceback
from itertools import ifilterfalse

def hex_reverse(data):
    return ''.join(reversed([data[i:i+2] for i in xrange(0, len(data), 2)]))

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

formats = defaultdict(lambda:(signed, thresh(10000)),{
    'status_word': (unsigned, lambda a,b: bin(a ^ b).count('1') > 2), 
    'op_mode_display': (unsigned, lambda a,b: False), # ignore
    'timestamp': (unsigned, lambda a,b: False), # ignore
    'velocity': (signed, thresh(11000)),
    'actual_velocity': (signed, thresh(30000)),
})

context = deque(maxlen=20)
dump_mode = 0

def dump_context(v, context, mode):
    if mode is 1:
        for c in context:
            print(c)
    elif mode is 2:
        s = hex_reverse(v)
        test = frozenset(ifilterfalse(lambda t: '0000' == t, (s[i:i + 4] for i in range(0, len(s)-2, 2))))
        for c in context:
            o = c[-1][0]
            for i in range(0, len(o)-2, 2):
                if o[i:i + 4] in test:
                    print('**', o[i:i + 4],)
                    break
            print(c)
    else:
        print("Error: dump mode not set!")
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
                val = fmt(v, len(v), True)
                if k in last[i]:
                    old = last[i][k]
                    last[i][k]=val
                    if jump(val, old):
                        print('t: {}, i: {}, {} = 0x{} <> 0x{}, |{} - {}| = {}'.format(p[0],i, k, v, fmt(old, len(v), False) ,val,old, abs(val-old)))
                        print('#',p)
                        dump_context(v, context, dump_mode)
                else:
                    last[i][k]=val
        context.append(p)
    except:
        print(line,)
        traceback.print_exc()
        break
