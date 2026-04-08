from __future__ import print_function

_emergency_stop = False


def request_stop(reason="bumper"):
    global _emergency_stop
    if not _emergency_stop:
        _emergency_stop = True
        print("EMERGENCY STOP: {}.".format(reason))


def is_stop_requested():
    return _emergency_stop


def reset_stop():
    global _emergency_stop
    _emergency_stop = False