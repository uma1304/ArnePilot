from cereal.messaging import PubMaster, new_message

pm = PubMaster(['thermal'])

active = False
while 1:
  msg = new_message('thermal')
  msg.thermal.started = active
  pm.send('thermal', msg)
  print('Sent {}'.format(active), flush=True)
  input()
  active = not active