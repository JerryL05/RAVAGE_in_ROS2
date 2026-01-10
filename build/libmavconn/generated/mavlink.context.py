# -*- coding: utf-8 -*-

MAVLINK_V20_DIALECTS = ['common', 'ardupilotmega']

IGNORE_DIALECTS = "".split(';')

# Most interesting dialects
_COMMON = 'common'
_APM = 'ardupilotmega'

for dialect in IGNORE_DIALECTS:
    if dialect in MAVLINK_V20_DIALECTS:
        MAVLINK_V20_DIALECTS.remove(dialect)

# common should be first
MAVLINK_V20_DIALECTS.sort()
MAVLINK_V20_DIALECTS.remove(_COMMON)
MAVLINK_V20_DIALECTS.insert(0, _COMMON)

# APM dialect should be second
if _APM in MAVLINK_V20_DIALECTS:
    MAVLINK_V20_DIALECTS.remove(_APM)
    MAVLINK_V20_DIALECTS.insert(1, _APM)
