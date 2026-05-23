"""Placeholder camera frame for synthetic vision."""

from __future__ import annotations

from fleet_contracts.perception import CameraFrame

from fleet_sim.state import SimState


class SimCameraSource:
    def __init__(self, state: SimState) -> None:
        self._state = state

    def acquire_frame(self) -> CameraFrame:
        return CameraFrame(
            frame_id="sim_camera",
            stamp_unix_s=self._state.sim_time_s,
            width_px=640,
            height_px=480,
            pixel_format="mono8",
            opaque_handle=f"sim:t={self._state.sim_time_s}",
        )

    def is_available(self) -> bool:
        return True
