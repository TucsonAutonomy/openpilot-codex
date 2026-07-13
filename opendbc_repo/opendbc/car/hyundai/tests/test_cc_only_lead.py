import importlib.util
from pathlib import Path
import unittest


MODULE_PATH = Path(__file__).parents[1] / "cc_only_lead.py"
SPEC = importlib.util.spec_from_file_location("cc_only_lead", MODULE_PATH)
assert SPEC is not None and SPEC.loader is not None
cc_only_lead = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(cc_only_lead)
CcOnlyButtons = cc_only_lead.CcOnlyButtons
CcOnlyLeadController = cc_only_lead.CcOnlyLeadController


def update(controller, **kwargs):
  values = {
    "enabled": True,
    "lead_enabled": True,
    "cruise_active": True,
    "v_ego": 25.0,
    "lead_visible": True,
    "lead_distance": 30.0,
    "lead_rel_speed": -1.0,
    "speed_camera_target": 250.0 / 3.6,
    "speed_camera_distance": 0.0,
    "speed_bump_target": 250.0 / 3.6,
    "speed_bump_distance": 0.0,
    "curve_target": 250.0 / 3.6,
    "turn_target": 250.0 / 3.6,
    "turn_distance": 0.0,
    "speed_camera_enabled": False,
    "speed_bump_enabled": False,
    "curve_enabled": False,
    "turn_enabled": False,
    "brake_pressed": False,
    "gas_pressed": False,
    "brake_hold_active": False,
    "driver_button": CcOnlyButtons.NONE,
  }
  values.update(kwargs)
  return controller.update(**values)


class TestCcOnlyLeadController(unittest.TestCase):
  def test_disabled_never_sends_buttons(self):
    controller = CcOnlyLeadController()
    for _ in range(100):
      self.assertEqual(update(controller, enabled=False, lead_distance=10.0, lead_rel_speed=-10.0), CcOnlyButtons.NONE)

  def test_below_minimum_speed_never_sends_buttons(self):
    controller = CcOnlyLeadController()
    for _ in range(100):
      self.assertEqual(update(controller, v_ego=8.0, lead_distance=8.0, lead_rel_speed=-5.0), CcOnlyButtons.NONE)

  def test_confirmed_close_lead_reduces_set_speed(self):
    controller = CcOnlyLeadController()
    sent = [update(controller) for _ in range(80)]
    self.assertIn(CcOnlyButtons.SET_DECEL, sent)
    self.assertGreater(controller.reduction_steps, 0)

  def test_never_automatically_resumes_after_lead_clears(self):
    controller = CcOnlyLeadController()
    for _ in range(80):
      update(controller)
    reductions = controller.reduction_steps
    self.assertGreater(reductions, 0)

    sent = [update(controller, lead_visible=False) for _ in range(500)]
    self.assertNotIn(CcOnlyButtons.RES_ACCEL, sent)
    self.assertEqual(controller.reduction_steps, reductions)

  def test_critical_closing_lead_cancels_without_auto_resume(self):
    controller = CcOnlyLeadController()
    sent = [update(controller, lead_distance=18.0, lead_rel_speed=-8.0) for _ in range(30)]
    self.assertEqual(sent.count(CcOnlyButtons.CANCEL), 1)
    self.assertTrue(controller.cancel_latched)

    for _ in range(400):
      self.assertEqual(update(controller, lead_visible=False), CcOnlyButtons.NONE)

  def test_driver_override_relinquishes_and_clears_recovery_budget(self):
    controller = CcOnlyLeadController()
    for _ in range(80):
      update(controller)
    self.assertGreater(controller.reduction_steps, 0)

    self.assertEqual(update(controller, driver_button=CcOnlyButtons.SET_DECEL), CcOnlyButtons.NONE)
    self.assertEqual(controller.reduction_steps, 0)
    for _ in range(300):
      self.assertEqual(update(controller, lead_visible=False), CcOnlyButtons.NONE)

  def test_pedal_override_yields_immediately(self):
    for pedal in ("brake_pressed", "gas_pressed", "brake_hold_active"):
      with self.subTest(pedal=pedal):
        controller = CcOnlyLeadController()
        for _ in range(80):
          update(controller)
        self.assertGreater(controller.reduction_steps, 0)
        self.assertEqual(update(controller, **{pedal: True}), CcOnlyButtons.NONE)
        self.assertEqual(controller.reduction_steps, 0)

  def test_each_target_speed_source_can_request_set_decel(self):
    cases = (
      ("camera", "speed_camera_target", "speed_camera_enabled"),
      ("bump", "speed_bump_target", "speed_bump_enabled"),
      ("curve", "curve_target", "curve_enabled"),
      ("turn", "turn_target", "turn_enabled"),
    )
    for kind, target, toggle in cases:
      with self.subTest(kind=kind):
        controller = CcOnlyLeadController()
        sent = [update(controller, lead_enabled=False, lead_visible=False,
                       **{target: 70.0 / 3.6, toggle: True}) for _ in range(80)]
        self.assertIn(CcOnlyButtons.SET_DECEL, sent)
        self.assertNotIn(CcOnlyButtons.RES_ACCEL, sent)

  def test_disabled_target_speed_source_never_sends(self):
    controller = CcOnlyLeadController()
    sent = [update(controller, lead_enabled=False, lead_visible=False,
                   speed_camera_target=50.0 / 3.6) for _ in range(100)]
    self.assertEqual(set(sent), {CcOnlyButtons.NONE})

  def test_disabled_lower_target_does_not_mask_enabled_camera(self):
    controller = CcOnlyLeadController()
    sent = [update(controller, lead_enabled=False, lead_visible=False,
                   speed_camera_target=70.0 / 3.6, speed_camera_enabled=True,
                   curve_target=40.0 / 3.6, curve_enabled=False) for _ in range(80)]
    self.assertIn(CcOnlyButtons.SET_DECEL, sent)
    self.assertEqual(controller.speed_kind, "camera")

  def test_target_at_current_speed_does_not_reduce(self):
    controller = CcOnlyLeadController()
    sent = [update(controller, lead_enabled=False, lead_visible=False,
                   speed_camera_target=90.0 / 3.6,
                   speed_camera_enabled=True) for _ in range(100)]
    self.assertEqual(set(sent), {CcOnlyButtons.NONE})

  def test_source_switch_preserves_reduction_budget(self):
    controller = CcOnlyLeadController()
    for _ in range(80):
      update(controller, lead_enabled=False, lead_visible=False,
             speed_camera_target=70.0 / 3.6, speed_camera_enabled=True)
    reductions = controller.speed_reduction_steps
    self.assertGreater(reductions, 0)

    update(controller, lead_enabled=False, lead_visible=False,
           curve_target=60.0 / 3.6, curve_enabled=True)
    self.assertEqual(controller.speed_kind, "curve")
    self.assertEqual(controller.speed_reduction_steps, reductions)

  def test_target_speed_clear_never_resumes(self):
    controller = CcOnlyLeadController()
    for _ in range(80):
      update(controller, lead_enabled=False, lead_visible=False,
             speed_camera_target=70.0 / 3.6,
             speed_camera_enabled=True)
    self.assertGreater(controller.speed_reduction_steps, 0)

    sent = [update(controller, lead_enabled=False, lead_visible=False,
                   speed_camera_target=250.0 / 3.6) for _ in range(300)]
    self.assertNotIn(CcOnlyButtons.RES_ACCEL, sent)
    self.assertEqual(controller.speed_reduction_steps, 0)

  def test_late_speed_event_cancels_cruise(self):
    controller = CcOnlyLeadController()
    sent = [update(controller, lead_enabled=False, lead_visible=False,
                   speed_bump_target=60.0 / 3.6, speed_bump_distance=20.0,
                   speed_bump_enabled=True) for _ in range(30)]
    self.assertEqual(sent.count(CcOnlyButtons.CANCEL), 1)
    self.assertTrue(controller.cancel_latched)

  def test_curve_target_never_uses_event_cancel(self):
    controller = CcOnlyLeadController()
    sent = [update(controller, lead_enabled=False, lead_visible=False,
                   curve_target=40.0 / 3.6, curve_enabled=True) for _ in range(80)]
    self.assertNotIn(CcOnlyButtons.CANCEL, sent)
    self.assertIn(CcOnlyButtons.SET_DECEL, sent)


if __name__ == "__main__":
  unittest.main()
