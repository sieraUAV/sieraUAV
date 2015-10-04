#!/usr/bin/python


#TEST ALGO

from video_algo.algo_manage import *
from video_algo.arrow import *
from video_algo.blk_sqr import *
from video_algo.cross import *
from misc.misc import *
from misc.th_misc import Q_TX, Q_RX
import time

def test_th():

	manAlgo=algo_process()
	cross_algo=cross()
	blk_sqr_algo=blk_sqr()
	ALGO_SELEC=ALGOS.NONE

	while not manAlgo.boolStop:
		manAlgo.read()

		#READ transmission queue
		(new_msg, msg)=Q_TX.get_bis()
		if new_msg:
			ALGO_SELEC=msg

		#ALGO SELECTION
		if ALGO_SELEC==ALGOS.NONE:
			time.sleep(0.1)
		elif ALGO_SELEC==ALGOS.ARROW:
			ret_com=arrow_algo.processing(manAlgo)
			Q_RX.put_bis(ret_com)
		elif ALGO_SELEC==ALGOS.BLK_SQUR:
			ret_com=blk_sqr_algo.processing(manAlgo)
			Q_RX.put_bis(ret_com)
		elif ALGO_SELEC==ALGOS.CROSS:
			ret_com=cross_algo.processing(manAlgo)
			Q_RX.put_bis(ret_com)		
		else:
			print "ERROR: BAD VIDEO ALGO"
			time.sleep(0.1)


		#Display IHM
		manAlgo.display()

		#Check for keyboard events
		manAlgo.toucheAction()