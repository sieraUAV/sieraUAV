#!/usr/bin/python


#TEST ALGO

from video_algo.algo_manage import *
from video_algo.arrow import *

manAlgo=algo_process()
arrow_algo=arrow()

while not manAlgo.boolStop:
	manAlgo.read()

	#Process
	arrow_algo.processing(manAlgo)

	manAlgo.display()

	manAlgo.toucheAction()