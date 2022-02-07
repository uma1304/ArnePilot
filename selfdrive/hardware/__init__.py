import os

EON = os.path.isfile('/EON')
TICI = os.path.isfile('/TICI')
PC = not (EON or TICI)
