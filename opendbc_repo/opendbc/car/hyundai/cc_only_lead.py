import math


class CcOnlyButtons:
  NONE = 0
  RES_ACCEL = 1
  SET_DECEL = 2
  CANCEL = 4


class CcOnlyLeadController:
  """Vision-lead assist for cars with conventional cruise control only.

  This controller cannot command throttle or brakes. It only sends conservative
  SET- button taps. Since a CC-only car does not report its current set speed,
  this controller never sends RES+ automatically. A critical closing lead
  cancels conventional cruise and is never resumed automatically.
  """

  LEAD_CONFIRM_FRAMES = 30       # 0.3 s at 100 Hz
  CRITICAL_CONFIRM_FRAMES = 10   # 0.1 s at 100 Hz
  SET_INTERVAL_FRAMES = 25       # at most 4 SET- taps per second
  SET_INTERVAL_URGENT_FRAMES = 15
  MAX_REDUCTION_STEPS = 40

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
    self.cancel_latched = False

  def configure(self, time_gap_s: float) -> None:
    self.time_gap_s = float(min(max(time_gap_s, 1.5), 3.0))

  def reset(self) -> None:
    self.lead_frames = 0
    self.no_lead_frames = 0
    self.filtered_distance = 0.0
    self.filtered_rel_speed = 0.0
    self.reduction_steps = 0
    self.cancel_latched = False

  def _valid_lead(self, visible: bool, distance: float, rel_speed: float) -> bool:
    return visible and math.isfinite(distance) and math.isfinite(rel_speed) and 2.0 < distance < 160.0

  def update(self, *, enabled: bool, cruise_active: bool, v_ego: float,
             lead_visible: bool, lead_distance: float, lead_rel_speed: float,
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
      self.last_button_frame = self.frame
      return CcOnlyButtons.NONE

    # After a critical cancel, never send RES+. Wait for the factory SET lamp to
    # turn off before the controller can be armed by the driver again.
    if self.cancel_latched:
      return CcOnlyButtons.NONE

    if v_ego * 3.6 < self.min_speed_kph:
      return CcOnlyButtons.NONE

    lead_valid = self._valid_lead(lead_visible, lead_distance, lead_rel_speed)
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
        self.last_button_frame = self.frame
        return CcOnlyButtons.CANCEL

      needs_reduction = distance < desired_gap or (ttc < 7.0 and rel_speed < -0.8)
      urgent = ttc < 4.0 or headway < 1.2
      interval = self.SET_INTERVAL_URGENT_FRAMES if urgent else self.SET_INTERVAL_FRAMES
      if (self.lead_frames >= self.LEAD_CONFIRM_FRAMES and needs_reduction and
          self.reduction_steps < self.MAX_REDUCTION_STEPS and
          self.frame - self.last_button_frame >= interval):
        self.reduction_steps += 1
        self.last_button_frame = self.frame
        return CcOnlyButtons.SET_DECEL

    return CcOnlyButtons.NONE
