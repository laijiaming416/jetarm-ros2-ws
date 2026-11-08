"""Bounded contact-candidate detection and position holding; no force sensor."""
import json
import statistics
import time


class GripFault(RuntimeError):
    pass


def sample(arm):
    p = arm.read_position(10)
    # read_temp 内部已做 4 次重试（50/100/200ms），这里不再叠加重试。
    t = arm.read_temp(10)
    load = arm.read_load_state(10)
    if type(p) is not int or not 0 <= p <= 1000:
        raise GripFault("missing_or_invalid_position")
    if type(t) is not int or t < 0:
        raise GripFault("temperature_feedback_missing_or_invalid")
    if t >= 80:
        raise GripFault("temperature_stop")
    if load != 1:
        raise GripFault("gripper_not_enabled")
    return p, t


def disable(arm):
    arm.unload(10)
    time.sleep(0.2)
    if arm.read_load_state(10) != 0:
        arm.unload(10)


def open_verified(arm, target):
    """Require feedback at the open position before moving the arm."""
    try:
        sample(arm)
        time.sleep(0.2)  # keep separate from load/read transactions
        arm.move({10: target}, 1200)
        time.sleep(1.6)
        for attempt in range(2):
            p, _ = sample(arm)
            if abs(p - target) <= 25:
                return p
            if attempt == 0:
                time.sleep(0.2)
                arm.move({10: target}, 1200)
                time.sleep(1.6)
        raise GripFault("open_target_not_reached")
    except BaseException:
        disable(arm)
        raise


def close_and_hold(arm, open_pos, close_pos):
    """Return a contact candidate, NOT proof of gripping or force control.

    Monitor only during this bounded call. No background thermal guard is
    installed; callers retaining a payload need their own continuous monitoring.
    Fault release is suitable here only because the test object rests on a table.
    """
    try:
        before, _ = sample(arm)
        if abs(before - open_pos) > 25:
            raise GripFault("not_open_before_closure")
        if close_pos - open_pos < 100:
            raise GripFault("invalid_close_range")
        time.sleep(0.2)
        arm.move({10: close_pos}, 1200)
        history = []
        for i in range(20):
            time.sleep(0.2)
            p, t = sample(arm)
            history.append(p)
            if i >= 7 and max(history[-4:]) - min(history[-4:]) <= 3:
                break
        else:
            raise GripFault("closure_not_stable")
        contact = int(round(statistics.median(history[-4:])))
        if contact - before < 30:
            raise GripFault("no_closing_motion")
        candidate = close_pos - contact >= 25
        # Remove sustained position error even on an empty close.
        arm.move({10: contact}, 800)
        time.sleep(0.2)
        arm.move({10: contact}, 800)
        temperatures = []
        for _ in range(10):
            time.sleep(0.2)
            actual, temperature = sample(arm)
            temperatures.append(temperature)
            if abs(actual - contact) > 8:
                raise GripFault("hold_position_not_stable")
        result = {"candidate": candidate, "before": before,
                  "contact_position": contact, "hold_target": contact,
                  "temperatures_c": temperatures, "closing_positions": history,
                  "physical_grip_verified": False}
        arm.gripper_hold_result = result
        print("GRIPPER_HOLD", json.dumps(result), flush=True)
        return contact, None if candidate else "empty_or_contact_too_near_closed_target"
    except BaseException:
        disable(arm)
        raise
