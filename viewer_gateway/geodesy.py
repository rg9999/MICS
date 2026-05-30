"""ENU (local sim frame) <-> geodetic (WGS84) transforms — PRD viewer §6.4.

The single most common integration bug ("everything is in the ocean off
Africa") is a datum mismatch, so this lives in exactly one place and the
inverse exists purely so a round-trip unit test can pin it down (PRD §12).

Frames:
  * ENU: east/north/up metres relative to a datum (lat0, lon0, alt0).
  * geodetic: WGS84 latitude/longitude (degrees) + ellipsoidal height (metres).

Cesium consumes geodetic; the sim/autonomy work in ENU. We convert once here.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

# WGS84 ellipsoid
_A = 6378137.0                      # semi-major axis (m)
_F = 1.0 / 298.257223563            # flattening
_E2 = _F * (2.0 - _F)               # first eccentricity squared
_B = _A * (1.0 - _F)               # semi-minor axis
_EP2 = (_A * _A - _B * _B) / (_B * _B)  # second eccentricity squared


@dataclass(frozen=True)
class Datum:
    """ENU origin. lat/lon in degrees, alt in metres (ellipsoidal)."""

    lat: float
    lon: float
    alt: float = 0.0


def _geodetic_to_ecef(lat_deg: float, lon_deg: float, alt: float) -> np.ndarray:
    lat = np.radians(lat_deg)
    lon = np.radians(lon_deg)
    sl, cl = np.sin(lat), np.cos(lat)
    so, co = np.sin(lon), np.cos(lon)
    n = _A / np.sqrt(1.0 - _E2 * sl * sl)
    x = (n + alt) * cl * co
    y = (n + alt) * cl * so
    z = (n * (1.0 - _E2) + alt) * sl
    return np.array([x, y, z], dtype=float)


def _ecef_to_geodetic(ecef: np.ndarray) -> tuple[float, float, float]:
    """Closed-form Bowring conversion ECEF -> (lat_deg, lon_deg, alt)."""
    x, y, z = float(ecef[0]), float(ecef[1]), float(ecef[2])
    lon = np.arctan2(y, x)
    p = np.hypot(x, y)
    if p < 1e-9:  # at a pole
        lat = np.pi / 2.0 if z >= 0 else -np.pi / 2.0
        alt = abs(z) - _B
        return float(np.degrees(lat)), float(np.degrees(lon)), float(alt)
    theta = np.arctan2(z * _A, p * _B)
    st, ct = np.sin(theta), np.cos(theta)
    lat = np.arctan2(z + _EP2 * _B * st**3, p - _E2 * _A * ct**3)
    sl = np.sin(lat)
    n = _A / np.sqrt(1.0 - _E2 * sl * sl)
    alt = p / np.cos(lat) - n
    return float(np.degrees(lat)), float(np.degrees(lon)), float(alt)


def _ecef_to_enu_rotation(lat_deg: float, lon_deg: float) -> np.ndarray:
    """Rotation mapping an ECEF delta into the local ENU frame at the datum."""
    lat = np.radians(lat_deg)
    lon = np.radians(lon_deg)
    sl, cl = np.sin(lat), np.cos(lat)
    so, co = np.sin(lon), np.cos(lon)
    return np.array([
        [-so,        co,       0.0],
        [-sl * co,  -sl * so,  cl],
        [cl * co,    cl * so,  sl],
    ], dtype=float)


class Transformer:
    """Caches the datum ECEF + rotation so per-point conversion is cheap."""

    def __init__(self, datum: Datum):
        self.datum = datum
        self._origin_ecef = _geodetic_to_ecef(datum.lat, datum.lon, datum.alt)
        self._r = _ecef_to_enu_rotation(datum.lat, datum.lon)  # ECEF->ENU
        self._rt = self._r.T                                    # ENU->ECEF

    def enu_to_geodetic(self, enu) -> tuple[float, float, float]:
        """ENU metres -> (lat_deg, lon_deg, alt_m)."""
        enu = np.asarray(enu, dtype=float).reshape(3)
        ecef = self._origin_ecef + self._rt @ enu
        return _ecef_to_geodetic(ecef)

    def geodetic_to_enu(self, lat_deg: float, lon_deg: float, alt: float) -> np.ndarray:
        """(lat_deg, lon_deg, alt_m) -> ENU metres. Inverse of enu_to_geodetic."""
        ecef = _geodetic_to_ecef(lat_deg, lon_deg, alt)
        return self._r @ (ecef - self._origin_ecef)

    def enu_to_lonlatalt(self, enu) -> list[float]:
        """Cesium-friendly order: [lon_deg, lat_deg, alt_m]."""
        lat, lon, alt = self.enu_to_geodetic(enu)
        return [lon, lat, alt]
