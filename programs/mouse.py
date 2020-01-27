#!/usr/bin/env python

import struct


# * Event types
EV_REL = 0x02

# * Relative axes
REL_X = 0x00
REL_Y = 0x01

EVENT_FORMAT = "llHHi"  # long, long, unsigned short, unsigned short, signed int
EVENT_SIZE = struct.calcsize(EVENT_FORMAT)


def mouse_example(mouse_path):
    with open(mouse_path, "rb") as file:
        event = file.read(EVENT_SIZE)
        while event:
            (tv_sec, tv_usec, type_, code, value) = struct.unpack(EVENT_FORMAT, event)
            if type_ == EV_REL:
                if code == REL_X:
                    print(value)
                elif code == REL_Y:
                    print(value)
            event = file.read(EVENT_SIZE)

if __name__ == '__main__':
    import sys
    if len(sys.argv) != 2:
        print 'Usage %s /dev/input/<path-to-event-mouse>' % sys.argv[0]
        quit()
    mouse_example(sys.argv[1])
