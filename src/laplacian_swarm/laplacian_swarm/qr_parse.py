"""Parse the compact QR JSON payload into a mission dict, with full schema/
limit validation (Madde 9, SW-15/SW-16).

Wire format (verified against the actual baked QR textures in
PX4-Autopilot/.../models/qr_codes - NOT the idealized schema in
ARCHITECTURE_DETAILED.md SS15, which uses different keys and includes a
"checksum"/"version" field that the real, physically-deployed QR codes do
not carry):
{"q":4,"f":"OK","i":15.5,"p":0,"r":0,"b":3,
 "sa":1,"sd":1,"sr":"KIRMIZI","sb":5,"n1":5,"n2":6,"n3":0}

There is also no "spacing" field in any of the 6 real QR payloads - spacing
stays manifest/mission_trigger-sourced, not QR-sourced. Since there is no
checksum field to validate, the role checksum would play (rejecting a
single corrupted OCR/zbar misread) is instead carried by the caller's
multi-frame confirmation (see swarm_agent_node's READ CONFIRM sub-state) -
this parser only validates that one frame is internally well-formed.
"""
import json
from laplacian_swarm import field, formations

VALID_COLORS = ('KIRMIZI', 'MAVI')
PITCH_ROLL_LIMIT_DEG = 30.0
WAIT_MAX_S = 30.0
SPLIT_WAIT_MAX_S = 60.0


def parse(payload, team_id, num_uavs=3):
    """Return a validated mission dict, or None if the payload is malformed,
    out of an enum/limit, or otherwise not safe to act on."""
    try:
        d = json.loads(payload)
    except Exception:
        return None
    if not isinstance(d, dict):
        return None

    try:
        qr_id = int(d['q'])
        formation = formations.strict_by_name(d.get('f', ''))
        altitude = float(d['i'])
        pitch_deg = float(d.get('p', 0.0))
        roll_deg = float(d.get('r', 0.0))
        wait_s = float(d.get('b', 0.0))
        split = bool(d.get('sa', 0))
        split_id = int(d.get('sd', 255))
        split_color = str(d.get('sr', '')).strip().upper()
        split_wait = float(d.get('sb', 0.0))
        n1, n2, n3 = int(d.get('n1', 0)), int(d.get('n2', 0)), int(d.get('n3', 0))
    except (KeyError, TypeError, ValueError):
        return None

    if qr_id not in field.QR_MAP:
        return None
    if formation is None:
        return None
    if not (field.GEOFENCE_MIN_ALT_M <= altitude <= field.GEOFENCE_MAX_ALT_M):
        return None
    if abs(pitch_deg) > PITCH_ROLL_LIMIT_DEG or abs(roll_deg) > PITCH_ROLL_LIMIT_DEG:
        return None
    if not (0.0 <= wait_s <= WAIT_MAX_S):
        return None
    if split:
        if not (0 <= split_id < num_uavs):
            return None
        if split_color not in VALID_COLORS:
            return None
        if not (0.0 <= split_wait <= SPLIT_WAIT_MAX_S):
            return None
    for nxt_qr in (n1, n2, n3):
        if nxt_qr != 0 and nxt_qr not in field.QR_MAP:
            return None

    nxt = {1: n1, 2: n2, 3: n3}
    return {
        'qr_id':       qr_id,
        'formation':   formation,
        'altitude':    altitude,
        'pitch_deg':   pitch_deg,
        'roll_deg':    roll_deg,
        'wait_s':      wait_s,
        'split':       split,
        'split_id':    split_id,
        'split_color': split_color,
        'split_wait':  split_wait,
        'next':        nxt.get(team_id, 0),
    }
