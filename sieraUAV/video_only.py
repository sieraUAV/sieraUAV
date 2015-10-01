#!/usr/bin/python
import sys
sys.path.append('./')
from video_th import *
from testalgo import test_th
from misc.th_misc import Q_TX, Q_RX
from video_algo.algo_manage import ALGOS

Q_TX.put_bis(ALGOS.BLK_SQUR)
test_th()