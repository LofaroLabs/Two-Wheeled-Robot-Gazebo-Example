# from ctypes import *
from ctypes import Structure,c_uint16,c_double,c_ubyte,c_uint32,c_int16

class H_REF(Structure):
    _pack_ = 1
    _fields_ = [("ref",    c_double*2)]

class H_TIME(Structure):
    _pack_ = 1
    _fields_ = [("sim",    c_double*1)]
