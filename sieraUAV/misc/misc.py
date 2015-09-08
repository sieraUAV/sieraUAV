#!/usr/bin/python

"""
@package This package implement basic tools (like enum)
"""

"""
Function to create a basic enum 
"""
def enum(*sequential, **named):
    enums = dict(zip(sequential, range(len(sequential))), **named)
    return type('Enum', (), enums)
