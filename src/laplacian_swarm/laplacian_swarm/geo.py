"""Frame helpers.

gz world is ENU:  X=East, Y=North, Z=Up.
PX4 local is NED: x=North, y=East, z=Down, origin = each drone's own spawn.
"""
import math

EARTH_R = 6378137.0  # WGS84 equatorial radius (m), flat-earth approx ok at field scale


def wgs84_to_enu(lat_deg, lon_deg, alt_m, origin_lat_deg, origin_lon_deg, origin_alt_m):
    """WGS84 point -> local tangent-plane ENU around an origin (Madde 6 FIELD_ENU).
    Equirectangular approximation: adequate at field scale (cm-level error
    over tens of meters), well inside GNSS/baro uncertainty (architecture §6.10)."""
    lat0 = math.radians(origin_lat_deg)
    dlat = math.radians(lat_deg - origin_lat_deg)
    dlon = math.radians(lon_deg - origin_lon_deg)
    north = dlat * EARTH_R
    east = dlon * EARTH_R * math.cos(lat0)
    up = alt_m - origin_alt_m
    return east, north, up


def gz_to_ned(target_gz, spawn_gz):
    """Global gz-ENU point -> this drone's local NED (about spawn_gz)."""
    n = target_gz[1] - spawn_gz[1]          # North = dY
    e = target_gz[0] - spawn_gz[0]          # East  = dX
    d = -(target_gz[2] - spawn_gz[2])       # Down  = -dZ
    return n, e, d


def local_ned_to_gz(local_ned, spawn_gz):
    """This drone's local NED (x,y,z) -> global gz-ENU point."""
    n, e, d = local_ned
    return (spawn_gz[0] + e, spawn_gz[1] + n, spawn_gz[2] - d)


def heading_to(cur_gz, tgt_gz):
    """NED yaw (rad) pointing from cur to tgt (gz-ENU xy)."""
    de = tgt_gz[0] - cur_gz[0]              # East
    dn = tgt_gz[1] - cur_gz[1]              # North
    if abs(de) < 1e-6 and abs(dn) < 1e-6:
        return 0.0
    return math.atan2(de, dn)              # NED yaw measured from North toward East


def rotate_body_to_ned(forward, right, yaw):
    """Body offset (forward, right) at heading yaw -> NED (dN, dE)."""
    dn = forward * math.cos(yaw) - right * math.sin(yaw)
    de = forward * math.sin(yaw) + right * math.cos(yaw)
    return dn, de


def project_ground(
        offset_x, offset_y, altitude_agl, roll, pitch, yaw,
        half_fov, aspect_y, cam_fwd=0.0):
    """Madde 10: down-facing camera pixel offset -> ground-plane (dN, dE)
    from the vehicle, using full attitude (roll/pitch), not just yaw.
    A camera that nominally boresights straight down doesn't actually
    point straight down once the vehicle tilts to accelerate laterally -
    ignoring that skews the ground intersection by meters at typical
    SPLIT/READ altitudes (the bug behind zones never locking precisely).
    Reduces to the old yaw-only rotate_body_to_ned at roll=pitch=0."""
    bf = math.tan(offset_y * half_fov) * aspect_y    # body-frame ray:
    br = math.tan(offset_x * half_fov)               # (forward, right, down)
    bd = 1.0
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    # standard body(FRD)->NED DCM (roll-pitch-yaw Euler), applied to the ray
    n = (cy * cp) * bf + (cy * sp * sr - sy * cr) * br + (cy * sp * cr + sy * sr) * bd
    e = (sy * cp) * bf + (sy * sp * sr + cy * cr) * br + (sy * sp * cr - cy * sr) * bd
    d = -sp * bf + (cp * sr) * br + (cp * cr) * bd
    d = max(d, 0.05)                                  # ray must point groundward
    s = altitude_agl / d
    return s * n + cam_fwd * cy, s * e + cam_fwd * sy
