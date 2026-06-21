"""Formation slots in BODY frame (forward +ahead, right +starboard).
The agent rotates these into NED by the formation heading (-> formation rotation
toward the next QR). Drone 1 = apex/reference at (0,0); 0 and 2 symmetric.
Distances scale with `spacing` (QR 'ajanlar arasi mesafe'). Tune to Sekil 3.

Sekil 3:  OK / OKBASI = arrow wedge (apex forward, tight)
          V                = open V       (apex forward, wide arms)
          CIZGI / line     = abreast across track
"""
import math

V = 0
LINE = 1
ARROW = 2
NONE = 255

_BY_NAME = {
    'V': V,
    'CIZGI': LINE, 'ÇIZGI': LINE, 'CİZGİ': LINE, 'LINE': LINE, 'CZ': LINE,
    'OK': ARROW, 'OKBASI': ARROW, 'OKBAŞI': ARROW, 'ARROW': ARROW,
}


def by_name(s):
    """Lenient lookup with a V fallback - for callers that always need SOME
    formation (e.g. agent boot default), not for validating untrusted data."""
    return _BY_NAME.get(str(s).strip().upper(), V)


def strict_by_name(s):
    """Madde 9 (formation enum validation): None on an unrecognized string -
    a real QR ('CZ' = CIZGI) silently mapping to V via the lenient lookup is
    exactly the bug this exists to catch, not something to paper over."""
    return _BY_NAME.get(str(s).strip().upper())


def slot(formation, slot_id, spacing):
    """Return (forward, right) body-frame offset for this manifest slot_id
    (Madde 7: fixed uav_id -> slot_id, NOT a per-leg dynamic ranking)."""
    d = float(spacing)
    table = {
        LINE:  [(0.0, -d),         (0.0, 0.0), (0.0, +d)],          # — abreast
        V:     [(-d, -d),          (0.0, 0.0), (-d, +d)],           # open V, wide
        ARROW: [(-1.2 * d, -0.6 * d), (0.0, 0.0), (-1.2 * d, +0.6 * d)],  # tight wedge
    }
    rows = table.get(formation)
    if rows is None or not (0 <= slot_id < len(rows)):
        return (0.0, 0.0)
    return rows[slot_id]


def axis(heading):
    """Shared forward/right unit vectors for a given formation heading
    (architecture §13: slot_e = ref_e + fwd*sin(h) + right*cos(h), etc)."""
    return (math.sin(heading), math.cos(heading)), (math.cos(heading), -math.sin(heading))


def slot_target(formation, slot_id, spacing, reference_xy, heading):
    """FIELD-frame (E, N) target for this slot, given a heading/reference
    shared identically by every agent (Madde 7 SW-09: no live peer/own
    position feeds into this - only static formation/mission inputs)."""
    fb, rb = slot(formation, slot_id, spacing)
    fwd, right = axis(heading)
    return (reference_xy[0] + fb * fwd[0] + rb * right[0],
            reference_xy[1] + fb * fwd[1] + rb * right[1])
