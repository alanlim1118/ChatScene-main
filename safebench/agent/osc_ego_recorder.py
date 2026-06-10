"""
FPV + BEV ego video recording for ScenarioRunner external-control workflows.

Extracted/adapted from PythonAPI/examples/recorder_osc.py.
"""

from __future__ import annotations

import os
import queue
import shutil
import subprocess
import threading
from datetime import datetime
from typing import Any, Dict, Optional


def check_ffmpeg_available() -> None:
    if shutil.which("ffmpeg") is None:
        raise RuntimeError(
            "ffmpeg not found on PATH. Install ffmpeg to use --record "
            "(e.g. apt install ffmpeg)."
        )


class AsyncFFmpegWriter:
    def __init__(self, out_path: str, w: int, h: int, fps: int, crf: int = 23, qsize: int = 96):
        self.q: queue.Queue = queue.Queue(maxsize=qsize)
        self.proc = subprocess.Popen(
            [
                "ffmpeg",
                "-y",
                "-hide_banner",
                "-loglevel",
                "error",
                "-f",
                "rawvideo",
                "-pix_fmt",
                "bgra",
                "-s",
                f"{w}x{h}",
                "-r",
                str(fps),
                "-i",
                "-",
                "-an",
                "-c:v",
                "libx264",
                "-crf",
                str(crf),
                "-pix_fmt",
                "yuv420p",
                out_path,
            ],
            stdin=subprocess.PIPE,
            stderr=subprocess.DEVNULL,
        )
        self.running = True
        threading.Thread(target=self._worker, daemon=True).start()

    def write(self, raw: bytes) -> None:
        try:
            self.q.put_nowait(raw)
        except queue.Full:
            pass

    def _worker(self) -> None:
        while self.running:
            item = self.q.get()
            if item is None:
                break
            try:
                if self.proc.stdin:
                    self.proc.stdin.write(item)
            except Exception:
                break

    def close(self) -> None:
        self.running = False
        try:
            self.q.put_nowait(None)
        except Exception:
            pass
        try:
            if self.proc.stdin:
                self.proc.stdin.close()
            self.proc.wait(timeout=5)
        except Exception:
            pass


class EgoVideoRecorder:
    """Records FPV (attached) and BEV (top-down follow) to MP4 via ffmpeg."""

    _REL_FPV = None  # set in start() from carla_mod

    def __init__(
        self,
        carla_mod: Any,
        world: Any,
        ego: Any,
        outdir: str,
        prefix: str,
        *,
        fps: int = 15,
        width: int = 1280,
        height: int = 720,
        bev_height: float = 50.0,
    ):
        self._carla = carla_mod
        self._world = world
        self._ego = ego
        self._outdir = outdir
        self._prefix = prefix
        self._fps = fps
        self._width = width
        self._height = height
        self._bev_height = bev_height

        self._writers: Dict[str, AsyncFFmpegWriter] = {}
        self._cams: Dict[str, Any] = {}
        self._active = False
        self.scene_dir: Optional[str] = None
        self._map_name: Optional[str] = None
        self._timestamp: Optional[str] = None

    @property
    def active(self) -> bool:
        return self._active

    def _outpath(self, view: str) -> str:
        assert self._map_name is not None and self._timestamp is not None
        return os.path.join(
            self.scene_dir,
            f"{self._prefix}__{self._map_name}__{view}__{self._timestamp}.mp4",
        )

    def start(self) -> str:
        if self._active:
            return self.scene_dir or ""

        check_ffmpeg_available()

        map_name = self._world.get_map().name.split("/")[-1]
        timestamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S_%f")[:-3]
        scene_dir = os.path.join(self._outdir, f"{self._prefix}__{map_name}__{timestamp}")
        os.makedirs(scene_dir, exist_ok=True)

        self._map_name = map_name
        self._timestamp = timestamp
        self.scene_dir = scene_dir

        rel_fpv = self._carla.Transform(
            self._carla.Location(1.6, -0.25, 1.35),
            self._carla.Rotation(pitch=-5),
        )

        bp = self._world.get_blueprint_library().find("sensor.camera.rgb")
        bp.set_attribute("image_size_x", str(self._width))
        bp.set_attribute("image_size_y", str(self._height))
        bp.set_attribute("sensor_tick", str(1.0 / self._fps))
        bp.set_attribute("fov", "110")

        def on_image(img: Any, view: str) -> None:
            writer = self._writers.get(view)
            if writer is not None:
                writer.write(img.raw_data)

        def spawn_cam(view: str, tf: Any, attach_to: Any = None) -> None:
            self._writers[view] = AsyncFFmpegWriter(
                self._outpath(view), self._width, self._height, self._fps
            )
            if attach_to is None:
                cam = self._world.spawn_actor(bp, tf)
            else:
                cam = self._world.spawn_actor(bp, tf, attach_to=attach_to)
            cam.listen(lambda img, v=view: on_image(img, v))
            self._cams[view] = cam

        ego_tf = self._ego.get_transform()
        spawn_cam("FPV", rel_fpv, attach_to=self._ego)
        spawn_cam(
            "BEV",
            self._carla.Transform(
                self._carla.Location(
                    ego_tf.location.x,
                    ego_tf.location.y,
                    ego_tf.location.z + self._bev_height,
                ),
                self._carla.Rotation(pitch=-90),
            ),
        )

        self._active = True
        print(f"[RECORD] started -> {scene_dir}", flush=True)
        return scene_dir

    def update_bev(self) -> None:
        if not self._active or "BEV" not in self._cams:
            return
        tf = self._ego.get_transform()
        self._cams["BEV"].set_transform(
            self._carla.Transform(
                self._carla.Location(
                    tf.location.x,
                    tf.location.y,
                    tf.location.z + self._bev_height,
                ),
                self._carla.Rotation(pitch=-90),
            )
        )

    def stop(self) -> None:
        if not self._active and not self._cams and not self._writers:
            return

        for cam in list(self._cams.values()):
            try:
                cam.stop()
                cam.destroy()
            except Exception:
                pass
        self._cams.clear()

        for writer in list(self._writers.values()):
            try:
                writer.close()
            except Exception:
                pass
        self._writers.clear()

        if self._active and self.scene_dir:
            print(f"[RECORD] saved to {self.scene_dir}", flush=True)
        self._active = False
