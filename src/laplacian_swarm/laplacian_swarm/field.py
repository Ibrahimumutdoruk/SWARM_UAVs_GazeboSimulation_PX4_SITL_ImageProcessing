"""Field survey - gz-world ENU coordinates (X=East, Y=North, Z=Up) of the QR
markers, taken from the simulation. QR index N corresponds to model qr_NN.
The competition shares QR positions before the run; update these if the world
changes. Heights are the marker plate height (~0.1 m); navigation uses xy only.
"""

QR_MAP = {
    1: (98.0,  0.0,  0.1),    # qr_11
    2: (112.0, 0.0,  0.1),    # qr_22
    3: (120.0, 10.0, 0.1),    # qr_33
    4: (112.0, 20.0, 0.1),    # qr_44
    5: (98.0,  20.0, 0.1),    # qr_55
    6: (90.0,  10.0, 0.1),    # qr_66
}

# First QR to fly to after takeoff (spec: "1 numarali QR")
FIRST_QR = 1

# Geofence (mirrors config/field.yaml) - used by qr_parse to validate a
# decoded mission's altitude before it is ever acted on.
GEOFENCE_MIN_ALT_M = 3.0
GEOFENCE_MAX_ALT_M = 30.0

# Madde 10: search polygon for an unlocated split color - a simple
# axis-aligned box covering the QR field plus margin (both known zone
# markers, mavi_alan (105,0) and kirmizi_alan (117,5), sit well inside it).
SEARCH_BOUNDS_E = (82.0, 128.0)
SEARCH_BOUNDS_N = (-15.0, 30.0)


def search_lane_target(t, lane_id, n_lanes, period_s=40.0):
    """Madde 10: divide SEARCH_BOUNDS into n_lanes east-west bands so
    multiple simultaneous searchers never overlap ("aktif arama
    cakismiyor") - lane_id sweeps north-south (triangle wave) at its
    band's center east coordinate."""
    min_e, max_e = SEARCH_BOUNDS_E
    min_n, max_n = SEARCH_BOUNDS_N
    n_lanes = max(1, n_lanes)
    band_w = (max_e - min_e) / n_lanes
    e = min_e + (lane_id % n_lanes + 0.5) * band_w
    x = (t % period_s) / period_s
    frac = 1.0 - abs(2.0 * x - 1.0)             # 0 -> 1 -> 0 triangle wave
    n = min_n + frac * (max_n - min_n)
    return e, n