"""Satellite imagery layer.

Fetches standard XYZ ("slippy map") tiles asynchronously with
``QNetworkAccessManager`` (no extra dependency), caches them on disk, and
places them in the scene using the online odom↔GPS georeference
(:class:`~mcs.core.geo.GeoReferencer`).

The layer stays hidden — and its checkbox disabled — until the
georeference is valid: without a trustworthy transform, imagery would be
misleading, which is worse than absent on an operator station.
"""

from __future__ import annotations

import logging
import math
from pathlib import Path

from PySide6.QtCore import Qt, QUrl
from PySide6.QtGui import QPixmap, QTransform
from PySide6.QtNetwork import QNetworkAccessManager, QNetworkReply, QNetworkRequest
from PySide6.QtWidgets import QGraphicsItemGroup, QGraphicsPixmapItem, QGraphicsScene

from mcs.config.settings import MapConfig
from mcs.core.geo import GeoFit, latlon_to_local_en, local_en_to_latlon, latlon_to_tile_xy, metres_per_pixel, tile_xy_to_latlon

_LOG = logging.getLogger(__name__)
_TILE_PX = 256


class TileLayer:
    """Manages the satellite tile items of one scene."""

    def __init__(self, scene: QGraphicsScene, cfg: MapConfig) -> None:
        self._scene = scene
        self._cfg = cfg
        self._group = QGraphicsItemGroup()
        self._group.setZValue(-100)
        self._group.setVisible(False)
        scene.addItem(self._group)
        self._net = QNetworkAccessManager()
        self._cache_dir = Path(cfg.tile_cache_dir)
        self._cache_dir.mkdir(parents=True, exist_ok=True)
        self._items: dict[tuple[int, int, int], QGraphicsPixmapItem] = {}
        self._pending: set[tuple[int, int, int]] = set()
        self._enabled = False
        self._current_zoom = -1

    # ------------------------------------------------------------------ API
    @property
    def enabled(self) -> bool:
        return self._enabled

    def set_enabled(self, enabled: bool) -> None:
        self._enabled = enabled
        self._group.setVisible(enabled)

    def update_view(self, fit: GeoFit | None, view_rect_world, px_per_m: float,
                    enu_scene: bool = False) -> None:
        """Ensure tiles covering the visible scene rect exist at a fitting zoom.

        ``enu_scene`` selects how a scene point maps to lat/lon: in ENU mode
        the scene axes ARE local east/north, so scene coords convert via the
        projection origin directly; otherwise the scene is the robot world
        frame and conversion goes through the georeference rotation."""
        if not self._enabled or fit is None:
            return
        self._enu_scene = enu_scene

        def scene_to_latlon(sx: float, sy: float) -> tuple[float, float]:
            if enu_scene:
                return local_en_to_latlon(sx, sy, fit.lat0, fit.lon0)
            return fit.world_to_latlon(sx, sy)

        self._scene_to_latlon = scene_to_latlon
        # Pick zoom so one tile pixel ~ one screen pixel.
        lat_c, _ = scene_to_latlon(view_rect_world.center().x(),
                                   view_rect_world.center().y())
        zoom = self._cfg.tile_max_zoom
        for z in range(3, self._cfg.tile_max_zoom + 1):
            if metres_per_pixel(lat_c, z) * px_per_m <= 1.2:
                zoom = z
                break

        # Show only the current zoom level: without this, tiles fetched at an
        # earlier zoom stay on top/underneath and the imagery looks blurry
        # after zooming in or out.
        if zoom != self._current_zoom:
            self._current_zoom = zoom
            for (z, _x, _y), item in self._items.items():
                item.setVisible(z == zoom)
            self._evict(keep_zoom=zoom)

        corners = [
            (view_rect_world.left(), view_rect_world.top()),
            (view_rect_world.right(), view_rect_world.top()),
            (view_rect_world.left(), view_rect_world.bottom()),
            (view_rect_world.right(), view_rect_world.bottom()),
        ]
        txs, tys = [], []
        for wx, wy in corners:
            lat, lon = scene_to_latlon(wx, wy)
            tx, ty = latlon_to_tile_xy(lat, lon, zoom)
            txs.append(tx)
            tys.append(ty)
        n = 2 ** zoom
        x_min = max(0, int(math.floor(min(txs))))
        x_max = min(n - 1, int(math.floor(max(txs))))
        y_min = max(0, int(math.floor(min(tys))))
        y_max = min(n - 1, int(math.floor(max(tys))))
        if (x_max - x_min + 1) * (y_max - y_min + 1) > 64:
            return  # zoomed out too far for this many tiles; skip quietly
        for tx in range(x_min, x_max + 1):
            for ty in range(y_min, y_max + 1):
                self._ensure_tile(zoom, tx, ty, fit)

    def _evict(self, keep_zoom: int, limit: int = 384) -> None:
        """Bound memory across zoom changes: drop hidden other-zoom tiles
        once the cache grows (disk cache makes re-adding cheap)."""
        if len(self._items) <= limit:
            return
        for key in [k for k in self._items if k[0] != keep_zoom]:
            item = self._items.pop(key)
            self._group.removeFromGroup(item)
            if item.scene() is not None:
                item.scene().removeItem(item)

    # ------------------------------------------------------------- internal
    def _ensure_tile(self, z: int, x: int, y: int, fit: GeoFit) -> None:
        key = (z, x, y)
        if key in self._items:
            self._place_tile(self._items[key], z, x, y, fit)
            return
        if key in self._pending:
            return
        cached = self._cache_dir / f"{z}_{x}_{y}.png"
        if cached.exists():
            pm = QPixmap(str(cached))
            if not pm.isNull():
                self._add_tile(key, pm, fit)
                return
        self._pending.add(key)
        url = self._cfg.tile_url.format(z=z, x=x, y=y)
        request = QNetworkRequest(QUrl(url))
        request.setHeader(QNetworkRequest.KnownHeaders.UserAgentHeader,
                          "BlueBoatMissionControl/1.0")
        reply = self._net.get(request)
        reply.finished.connect(lambda r=reply, k=key, f=fit: self._on_reply(r, k, f))

    def _on_reply(self, reply: QNetworkReply, key: tuple[int, int, int],
                  fit: GeoFit) -> None:
        self._pending.discard(key)
        data = bytes(reply.readAll())
        reply.deleteLater()
        if reply.error() != QNetworkReply.NetworkError.NoError or not data:
            _LOG.debug("Tile %s failed: %s", key, reply.errorString())
            return
        pm = QPixmap()
        if not pm.loadFromData(data):
            return
        (self._cache_dir / f"{key[0]}_{key[1]}_{key[2]}.png").write_bytes(data)
        self._add_tile(key, pm, fit)

    def _add_tile(self, key: tuple[int, int, int], pm: QPixmap, fit: GeoFit) -> None:
        item = QGraphicsPixmapItem(pm)
        # Strict PySide6 builds reject raw ints for enum parameters — the
        # previous `setTransformationMode(1)` raised TypeError on every tile,
        # which is why the satellite layer never appeared at all.
        item.setTransformationMode(Qt.TransformationMode.SmoothTransformation)
        self._group.addToGroup(item)
        item.setVisible(key[0] == self._current_zoom)
        self._items[key] = item
        self._place_tile(item, *key, fit)

    def _place_tile(self, item: QGraphicsPixmapItem, z: int, x: int, y: int,
                    fit: GeoFit) -> None:
        """Map the tile's 4 geo corners into world metres via the geo fit.

        Tiles are square in web-mercator, and locally (harbour scale) the
        world frame is a rotation + translation of local EN metres, so an
        affine placement of the NW corner + scale + rotation is accurate.
        """
        lat_nw, lon_nw = tile_xy_to_latlon(x, y, z)
        lat_c, _ = tile_xy_to_latlon(x + 0.5, y + 0.5, z)
        m_per_px = metres_per_pixel(lat_c, z)
        transform = QTransform()
        if getattr(self, "_enu_scene", False):
            # ENU scene (north-up fixed): the tile is axis-aligned, NW corner
            # at its east/north metres, no rotation — this is the QGC look.
            en, nn = latlon_to_local_en(lat_nw, lon_nw, fit.lat0, fit.lon0)
            transform.translate(en, nn)
            transform.scale(m_per_px, -m_per_px)  # +v south => -north
        else:
            # World-frame scene (before heading alignment): NW corner mapped
            # into world metres, tile rotated by the georef theta.
            wx, wy = fit.latlon_to_world(lat_nw, lon_nw)
            transform.translate(wx, wy)
            transform.rotateRadians(fit.theta)
            transform.scale(m_per_px, -m_per_px)
        item.setTransform(transform)
