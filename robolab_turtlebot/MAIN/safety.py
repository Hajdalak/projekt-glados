from __future__ import print_function

# Shared latched emergency-stop flag used by all modules.
_emergency_stop = False


def request_stop(reason="bumper"):
    """Latch the shared emergency stop and print the reason once."""
    global _emergency_stop
    if not _emergency_stop:
        _emergency_stop = True
        print("EMERGENCY STOP: {}.".format(reason))


def is_stop_requested():
    """Return the current state of the shared emergency stop."""
    return _emergency_stop


def reset_stop():
    """Clear the shared emergency stop flag."""
    global _emergency_stop
    _emergency_stop = False

