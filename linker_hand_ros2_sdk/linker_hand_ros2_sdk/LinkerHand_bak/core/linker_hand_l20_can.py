#!/usr/bin/env python3
import can
import time,sys,os
import threading
import numpy as np
from enum import Enum

class LinkerHandL20Can:
    def __init__(self,can_id, can_channel='can0', baudrate=1000000):
        pass