"""Recognize bounded, evidenced tracking interruptions; never infer a stop."""
import math


def tracking_reason(error):
    """Return the recoverable kind, or None; group peers must all be known."""
    if (not isinstance(error, dict) or error.get('code') != 'tracking_error'
            or error.get('stage') != 'tracking'
            or error.get('execution_outcome') not in {'partial', 'none'}):
        return None
    details = error.get('details') or {}
    if 'devices' in details:
        peers = details['devices']
        if not isinstance(peers, dict) or not peers:
            return None
        # Legacy Body reports only per-peer command fractions plus an explicit
        # all-arrival-timeouts aggregate. Preserve that existing recovery path.
        if (details.get('reason') == 'arrival_timeout'
                and details.get('group_all_arrival_timeouts') is True
                and all(isinstance(p, dict) and set(p) == {'command_fraction'} for p in peers.values())):
            return 'arrival_timeout'
        reasons = []
        for peer in peers.values():
            if not isinstance(peer, dict):
                return None
            if 'error' in peer:
                reason = tracking_reason(peer['error'])
                if reason is None:
                    return None
                reasons.append(reason)
            elif not isinstance(peer.get('result'), dict) or not isinstance(peer['result'].get('completed'), bool):
                return None
        if not reasons:
            return None
        return 'tracking_deviation' if 'tracking_deviation' in reasons else 'arrival_timeout'
    reason = details.get('reason')
    if reason == 'arrival_timeout':
        return reason
    if reason != 'tracking_deviation' or details.get('hold_command_sent') is not True:
        return None
    limits = details.get('tracking_limits') or {}
    values = [details.get('position_error_m'), details.get('rotation_error_rad'),
              limits.get('position_m'), limits.get('rotation_rad')]
    if not all(type(x) in (int, float) and math.isfinite(x) and x >= 0 for x in values):
        return None
    pos, rot, max_pos, max_rot = values
    if min(max_pos, max_rot) <= 0 or (pos <= max_pos and rot <= max_rot):
        return None
    return reason
