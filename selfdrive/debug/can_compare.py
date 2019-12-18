#!/usr/bin/env python3
import binascii
import os
import sys
from collections import defaultdict

import selfdrive.messaging as messaging
from common.realtime import sec_since_boot


def can_compare(bus=0, max_msg=None, addr="127.0.0.1"):
  logcan = messaging.sub_sock('can', addr=addr)

  start = sec_since_boot()
  msgs = defaultdict(list)
  canbus = int(os.getenv("CAN", bus))
  while sec_since_boot()-start < 5.0:
    can_recv = messaging.drain_sock(logcan, wait_for_one=True)
    for x in can_recv:
      for y in x.can:
        if y.src == canbus:
          msgs[y.address].append(y.dat)
  try:
    input("Change State and press Enter to continue...")
  except SyntaxError:
    pass
  start = sec_since_boot()
  msgs2 = defaultdict(list)
  while sec_since_boot()-start < 5.0:
    can_recv = messaging.drain_sock(logcan, wait_for_one=True)
    for x in can_recv:
      for y in x.can:
        if y.src == canbus:
          msgs2[y.address].append(y.dat)
  try:
    input("Change State back and press Enter to continue...")
  except SyntaxError:
    pass
  start = sec_since_boot()
  msgs3 = defaultdict(list)
  while sec_since_boot()-start < 5.0:
    can_recv = messaging.drain_sock(logcan, wait_for_one=True)
    for x in can_recv:
      for y in x.can:
        if y.src == canbus:
          msgs3[y.address].append(y.dat)
  dd = chr(27)
  for k,v in sorted(zip(msgs.keys(), map(lambda x: binascii.hexlify(x[-1]), msgs.values()))):
    try:
      if binascii.hexlify(msgs2.values()[list(msgs2).index(k)][-1]) != binascii.hexlify(msgs3.values()[list(msgs3).index(k)][-1]) and v == binascii.hexlify(msgs3.values()[list(msgs3).index(k)][-1]):
        dd +="%s %s\n" % ("%04X(%4d)" % (k,k), v.decode('ascii'))
        w = binascii.hexlify(msgs2.values()[msgs2.keys().index(k)][-1]).decode('ascii')
        dd +="%s %s\n" % ("%04X(%4d)" % (k,k), w.decode('ascii'))
    except ValueError:
      pass
  print(dd)


if __name__ == "__main__":
  if len(sys.argv) > 3:
    can_compare(int(sys.argv[1]), int(sys.argv[2]), sys.argv[3])
  elif len(sys.argv) > 2:
    can_compare(int(sys.argv[1]), int(sys.argv[2]))
  elif len(sys.argv) > 1:
    can_compare(int(sys.argv[1]))
  else:
    can_compare()
