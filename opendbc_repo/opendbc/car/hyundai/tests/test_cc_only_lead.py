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
    "cruise_active": True,
    "v_ego": 25.0,
    "lead_visible": True,
    "lead_distance": 30.0,
    "lead_rel_speed": -1.0,
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


if __name__ == "__main__":
  unittest.main()
