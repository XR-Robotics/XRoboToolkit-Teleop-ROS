import time
from transforms3d import _gohlketransforms as tg

__all__ = ["second_print",  "quat2rpy"]

_prev_time = None

def second_print(_str, interval=1):
    global _prev_time

    now = time.time()
    if _prev_time == None:
        _prev_time = now
        print(_str)
    elif (now - _prev_time) > interval:
        _prev_time = now
        print(_str)

def quat2rpy(quat):
    return tg.euler_from_quaternion(quat, axes='rzyx')
