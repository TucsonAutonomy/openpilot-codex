import math


class CcOnlyButtons:
  NONE = 0
  RES_ACCEL = 1
  SET_DECEL = 2
  CANCEL = 4


class CcOnlyLeadController:
  """Vision-lead and target-speed assist for conventional cruise control.

  This controller cannot command throttle or brakes. It only sends conservative
  SET- button taps. Optional recovery only restores SET- taps sent by this
  controller while factory cruise remains active. A critical closing lead or
  late speed event requests cruise cancel and is never resumed automatically.
  """

  LEAD_CONFIRM_FRAMES = 30       # 0.3 s at 100 Hz
  CRITICAL_CONFIRM_FRAMES = 10   # 0.1 s at 100 Hz
  SET_INTERVAL_FRAMES = 25       # at most 4 SET- taps per second
  SET_INTERVAL_URGENT_FRAMES = 15
  MAX_REDUCTION_STEPS = 40
  SPEED_CONFIRM_FRAMES = 20
  SPEED_CRITICAL_CONFIRM_FRAMES = 10
  SPEED_SET_INTERVAL_FRAMES = 50  # at most 2 target-speed SET- taps per second
  SPEED_MARGIN_KPH = 3.0
  SPEED_CRITICAL_EXCESS_KPH = 15.0
  SPEED_CRITICAL_TTC = 2.0
  MAX_SPEED_REDUCTION_STEPS = 30
  RECOVERY_CONFIRM_FRAMES = 300       # require 3.0 s of clear conditions
  RECOVERY_INTERVAL_FRAMES = 100      # at most one RES+ tap per second
  RECOVERY_HEADWAY_MARGIN_S = 0.5
  RECOVERY_MIN_REL_SPEED = -0.3
  MAX_RECOVERY_STEPS = 30

  def __init__(self, time_gap_s: float = 2.0, min_speed_kph: float = 30.0):
    self.time_gap_s = time_gap_s
    self.min_speed_kph = min_speed_kph
    self.frame = 0
    self.last_button_frame = -self.SET_INTERVAL_FRAMES
    self.lead_frames = 0
    self.no_lead_frames = 0
    self.filtered_distance = 0.0
    self.filtered_rel_speed = 0.0
    self.reduction_steps = 0
    self.speed_kind = ""
    self.speed_frames = 0
    self.speed_reduction_steps = 0
    self.recovery_frames = 0
    self.recovery_steps = 0
    self.cancel_latched = False

  def configure(self, time_gap_s: float) -> None:
    self.time_gap_s = float(min(max(time_gap_s, 1.5), 3.0))

  def reset(self) -> None:
    self.lead_frames = 0
    self.no_lead_frames = 0
    self.filtered_distance = 0.0
    self.filtered_rel_speed = 0.0
    self.reduction_steps = 0
    self._reset_speed_event()
    self._reset_recovery()
    self.cancel_latched = False

  def _reset_speed_event(self) -> None:
    self.speed_kind = ""
    self.speed_frames = 0
    self.speed_reduction_steps = 0

  def _reset_recovery(self) -> None:
    self.recovery_frames = 0
    self.recovery_steps = 0

  def _record_reduction(self) -> None:
    self.recovery_frames = 0
    self.recovery_steps = min(self.recovery_steps + 1, self.MAX_RECOVERY_STEPS)

  def _valid_speed_target(self, target_speed: float) -> bool:
    return math.isfinite(target_speed) and 20.0 <= target_speed * 3.6 <= 160.0

  def _valid_lead(self, visible: bool, distance: float, rel_speed: float) -> bool:
    return visible and math.isfinite(distance) and math.isfinite(rel_speed) and 2.0 < distance < 160.0

  def update(self, *, enabled: bool, lead_enabled: bool, cruise_active: bool, v_ego: float,
             lead_visible: bool, lead_distance: float, lead_rel_speed: float,
             speed_camera_target: float, speed_camera_distance: float,
             speed_bump_target: float, speed_bump_distance: float,
             curve_target: float, turn_target: float, turn_distance: float,
             speed_camera_enabled: bool, speed_bump_enabled: bool,
             curve_enabled: bool, turn_enabled: bool, auto_resume_enabled: bool,
             brake_pressed: bool, gas_pressed: bool, brake_hold_active: bool,
             driver_button: int) -> int:
    self.frame += 1

    if not enabled or not cruise_active:
      self.reset()
      return CcOnlyButtons.NONE

    # Any driver input owns the cruise state.
    if driver_button != CcOnlyButtons.NONE or brake_pressed or gas_pressed or brake_hold_active:
      self.reduction_steps = 0
      self.lead_frames = 0
      self.no_lead_frames = 0
      self._reset_speed_event()
      self._reset_recovery()
      self.last_button_frame = self.frame
      return CcOnlyButtons.NONE

    # After a critical cancel, never send RES+. Wait for the factory SET lamp to
    # turn off before the controller can be armed by the driver again.
    if self.cancel_latched:
      return CcOnlyButtons.NONE

    if v_ego * 3.6 < self.min_speed_kph:
      self.lead_frames = 0
      self._reset_speed_event()
      self._reset_recovery()
      return CcOnlyButtons.NONE

    if not auto_resume_enabled:
      self._reset_recovery()

    lead_valid = lead_enabled and self._valid_lead(lead_visible, lead_distance, lead_rel_speed)
    if lead_valid:
      if self.lead_frames == 0:
        self.filtered_distance = lead_distance
        self.filtered_rel_speed = lead_rel_speed
      else:
        alpha = 0.15
        self.filtered_distance += alpha * (lead_distance - self.filtered_distance)
        self.filtered_rel_speed += alpha * (lead_rel_speed - self.filtered_rel_speed)
      self.lead_frames += 1
      self.no_lead_frames = 0
    else:
      self.lead_frames = 0
      self.no_lead_frames += 1

    speed_targets = []
    if speed_camera_enabled and self._valid_speed_target(speed_camera_target):
      speed_targets.append(("camera", speed_camera_target * 3.6, speed_camera_distance))
    if speed_bump_enabled and self._valid_speed_target(speed_bump_target):
      speed_targets.append(("bump", speed_bump_target * 3.6, speed_bump_distance))
    if curve_enabled and self._valid_speed_target(curve_target):
      speed_targets.append(("curve", curve_target * 3.6, 0.0))
    if turn_enabled and self._valid_speed_target(turn_target):
      speed_targets.append(("turn", turn_target * 3.6, turn_distance))

    speed_target_valid = len(speed_targets) > 0
    if speed_target_valid:
      speed_kind, desired_speed_kph, _ = min(speed_targets, key=lambda target: target[1])
      if speed_kind != self.speed_kind:
        self.speed_kind = speed_kind
        self.speed_frames = 0
      self.speed_frames += 1
    else:
      self._reset_speed_event()

    if lead_valid:
      distance = self.filtered_distance
      rel_speed = self.filtered_rel_speed
      desired_gap = max(8.0, self.time_gap_s * v_ego)
      headway = distance / max(v_ego, 0.1)
      ttc = distance / -rel_speed if rel_speed < -0.5 else math.inf

      # Button-only control cannot guarantee deceleration. Cancel conventional
      # cruise when the closing situation is outside this limited controller's
      # useful range. The driver must brake.
      critical = ((headway < 0.75 and distance < desired_gap * 0.55) or
                  (ttc < 2.5 and rel_speed < -2.0))
      if self.lead_frames >= self.CRITICAL_CONFIRM_FRAMES and critical:
        self.cancel_latched = True
        self.reduction_steps = 0
        self._reset_recovery()
        self.last_button_frame = self.frame
        return CcOnlyButtons.CANCEL

    if speed_target_valid:
      critical_event = any(
        kind in ("camera", "bump", "turn") and distance > 0.0 and
        distance / max(v_ego, 0.1) < self.SPEED_CRITICAL_TTC and
        v_ego * 3.6 - target_kph > self.SPEED_CRITICAL_EXCESS_KPH
        for kind, target_kph, distance in speed_targets
      )
      if self.speed_frames >= self.SPEED_CRITICAL_CONFIRM_FRAMES and critical_event:
        self.cancel_latched = True
        self.speed_reduction_steps = 0
        self._reset_recovery()
        self.last_button_frame = self.frame
        return CcOnlyButtons.CANCEL

    if lead_valid:
      distance = self.filtered_distance
      rel_speed = self.filtered_rel_speed
      desired_gap = max(8.0, self.time_gap_s * v_ego)
      headway = distance / max(v_ego, 0.1)
      ttc = distance / -rel_speed if rel_speed < -0.5 else math.inf

      needs_reduction = distance < desired_gap or (ttc < 7.0 and rel_speed < -0.8)
      urgent = ttc < 4.0 or headway < 1.2
      interval = self.SET_INTERVAL_URGENT_FRAMES if urgent else self.SET_INTERVAL_FRAMES
      if (self.lead_frames >= self.LEAD_CONFIRM_FRAMES and needs_reduction and
          self.reduction_steps < self.MAX_REDUCTION_STEPS and
          self.frame - self.last_button_frame >= interval):
        self.reduction_steps += 1
        if auto_resume_enabled:
          self._record_reduction()
        self.last_button_frame = self.frame
        return CcOnlyButtons.SET_DECEL

    if speed_target_valid:
      speed_excess = v_ego * 3.6 - desired_speed_kph
      if (self.speed_frames >= self.SPEED_CONFIRM_FRAMES and
          speed_excess > self.SPEED_MARGIN_KPH and
          self.speed_reduction_steps < self.MAX_SPEED_REDUCTION_STEPS and
          self.frame - self.last_button_frame >= self.SPEED_SET_INTERVAL_FRAMES):
        self.speed_reduction_steps += 1
        if auto_resume_enabled:
          self._record_reduction()
        self.last_button_frame = self.frame
        return CcOnlyButtons.SET_DECEL

    lead_allows_recovery = not lead_enabled or not lead_visible
    if lead_valid:
      headway = self.filtered_distance / max(v_ego, 0.1)
      lead_allows_recovery = (headway >= self.time_gap_s + self.RECOVERY_HEADWAY_MARGIN_S and
                              self.filtered_rel_speed >= self.RECOVERY_MIN_REL_SPEED)

    recovery_allowed = (auto_resume_enabled and self.recovery_steps > 0 and
                        not speed_target_valid and lead_allows_recovery)
    if recovery_allowed:
      self.recovery_frames += 1
      if (self.recovery_frames >= self.RECOVERY_CONFIRM_FRAMES and
          self.frame - self.last_button_frame >= self.RECOVERY_INTERVAL_FRAMES):
        self.recovery_steps -= 1
        self.last_button_frame = self.frame
        if self.recovery_steps == 0:
          self.recovery_frames = 0
        return CcOnlyButtons.RES_ACCEL
    else:
      self.recovery_frames = 0

    return CcOnlyButtons.NONE
