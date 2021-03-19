from cereal.messaging import PubMaster, new_message

pm = PubMaster(['trafficModelControl'])

active = False
while 1:
  msg = new_message('trafficModelControl')
  msg.trafficModelControl.active = active
  pm.send('trafficModelControl', msg)
  print('Sent {}'.format(active), flush=True)
  input()
  active = not active
