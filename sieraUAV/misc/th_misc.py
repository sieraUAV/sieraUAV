#!/usr/bin/python
"""
@package This package contains the tools to control and communicate with threads
"""


import threading
import Queue

#THREAD CLASS DEF
class myThread (threading.Thread):
    def __init__(self, threadID, name, function):
        threading.Thread.__init__(self)
        self.threadID = threadID
        self.name = name
        self.function = function
    def run(self):
        print "Starting " + self.name
        self.function()
        print "EXIT " + self.name



#COMMUNICATION Definition

class Queue_cust(Queue.Queue):
	def __init__(self, nbritem):
		Queue.Queue.__init__(self, nbritem)

	def flush(self):
		
		while not self.empty():
			self.get()

	def put_bis(self, obj):

		#If cache full throw the last item
		if self.full():
			self.get()

		self.put(obj)





"""
Queue between main <--- video_th (max 10 items)

Use to send informations from image processing thread to the main thread
"""
Q_RX=Queue_cust(10)

"""
Queue between main ---> video_th (max 10 items)

Use to send command from main thread to image processing thread
"""
Q_TX=Queue_cust(10)
