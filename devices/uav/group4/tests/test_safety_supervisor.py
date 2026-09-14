import unittest

from agent.safety_supervisor import SafetyAction, SafetySupervisor, SafetyThresholds


def healthy_state(**overrides):
    state = {
        "pose": {"x_m": 1.0, "y_m": 1.0, "z_m": 1.0},
        "position_valid": True,
        "position_age_s": 0.0,
        "battery": {"remaining_percent": 80},
        "battery_age_s": 0.0,
        "fcu_link_age_s": 0.0,
        "coordinator_link_age_s": 0.0,
    }
    state.update(overrides)
    return state


def stabilized(supervisor, state=None):
    value = state or healthy_state()
    supervisor.evaluate(value, now_s=0.0, airborne=False)
    return supervisor.evaluate(value, now_s=2.1, airborne=False)


class SafetySupervisorTests(unittest.TestCase):
    def test_healthy_stable_state_allows_operation(self):
        supervisor = SafetySupervisor()
        stabilized(supervisor)
        decision = supervisor.evaluate(healthy_state(), now_s=2.2, airborne=False, accepting_new_task=True)
        self.assertEqual(SafetyAction.NONE, decision.action)

    def test_low_battery_rejects_takeoff(self):
        supervisor = SafetySupervisor()
        state = healthy_state(battery={"remaining_percent": 30})
        stabilized(supervisor, state)
        decision = supervisor.evaluate(state, now_s=2.2, airborne=False, accepting_new_task=True)
        self.assertEqual(SafetyAction.REJECT_NEW_TASK, decision.action)
        self.assertEqual("LOW_BATTERY", decision.code)

    def test_low_battery_returns_when_airborne_and_position_valid(self):
        supervisor = SafetySupervisor()
        state = healthy_state(battery={"remaining_percent": 20})
        stabilized(supervisor, state)
        decision = supervisor.evaluate(state, now_s=2.2, airborne=True)
        self.assertEqual(SafetyAction.RETURN_HOME, decision.action)

    def test_critical_battery_lands_when_position_valid(self):
        supervisor = SafetySupervisor()
        stabilized(supervisor)
        state = healthy_state(battery={"remaining_percent": 10})
        decision = supervisor.evaluate(state, now_s=2.2, airborne=True)
        self.assertEqual(SafetyAction.LAND_NOW, decision.action)

    def test_ground_rejection_is_not_latched(self):
        supervisor = SafetySupervisor()
        critical = supervisor.evaluate(
            healthy_state(battery={"remaining_percent": 10}),
            now_s=0.0,
            airborne=False,
            accepting_new_task=True,
        )
        self.assertEqual(SafetyAction.REJECT_NEW_TASK, critical.action)
        stabilized(supervisor)
        recovered = supervisor.evaluate(
            healthy_state(), now_s=2.2, airborne=False, accepting_new_task=True
        )
        self.assertEqual(SafetyAction.NONE, recovered.action)

    def test_position_loss_never_blindly_returns(self):
        supervisor = SafetySupervisor()
        state = healthy_state(position_valid=False, position_age_s=5.0)
        decision = supervisor.evaluate(state, now_s=5.0, airborne=True)
        self.assertEqual(SafetyAction.MANUAL_TAKEOVER, decision.action)
        self.assertNotEqual(SafetyAction.RETURN_HOME, decision.action)

    def test_position_recovery_requires_stability(self):
        supervisor = SafetySupervisor(SafetyThresholds(position_recovery_stable_s=2.0))
        first = supervisor.evaluate(healthy_state(), now_s=10.0, airborne=True)
        second = supervisor.evaluate(healthy_state(), now_s=11.0, airborne=True)
        third = supervisor.evaluate(healthy_state(), now_s=12.1, airborne=True)
        self.assertEqual(SafetyAction.HOLD_POSITION, first.action)
        self.assertEqual(SafetyAction.HOLD_POSITION, second.action)
        self.assertEqual(SafetyAction.NONE, third.action)

    def test_fcu_loss_requires_manual_takeover_in_flight(self):
        supervisor = SafetySupervisor()
        decision = supervisor.evaluate(healthy_state(fcu_link_age_s=4.0), now_s=4.0, airborne=True)
        self.assertEqual(SafetyAction.MANUAL_TAKEOVER, decision.action)
        self.assertEqual("FCU_LINK_LOST", decision.code)

    def test_coordinator_loss_returns_if_local_control_is_healthy(self):
        supervisor = SafetySupervisor()
        stabilized(supervisor)
        decision = supervisor.evaluate(healthy_state(coordinator_link_age_s=4.0), now_s=4.0, airborne=True)
        self.assertEqual(SafetyAction.RETURN_HOME, decision.action)
        self.assertEqual("COORDINATOR_LINK_LOST", decision.code)

    def test_emergency_action_is_latched_and_not_replaced(self):
        supervisor = SafetySupervisor()
        stabilized(supervisor)
        first = supervisor.evaluate(healthy_state(battery={"remaining_percent": 20}), now_s=3.0, airborne=True)
        second = supervisor.evaluate(healthy_state(coordinator_link_age_s=8.0), now_s=8.0, airborne=True)
        self.assertEqual("LOW_BATTERY_RETURN", first.code)
        self.assertEqual("LOW_BATTERY_RETURN", second.code)
        self.assertTrue(second.latched)

    def test_reset_only_after_confirmed_landing_allows_new_event(self):
        supervisor = SafetySupervisor()
        stabilized(supervisor)
        supervisor.evaluate(healthy_state(battery={"remaining_percent": 20}), now_s=3.0, airborne=True)
        supervisor.reset_after_safe_landing()
        decision = supervisor.evaluate(healthy_state(fcu_link_age_s=4.0), now_s=4.0, airborne=False)
        self.assertEqual("FCU_LINK_LOST", decision.code)
        self.assertEqual(SafetyAction.REJECT_NEW_TASK, decision.action)


if __name__ == "__main__":
    unittest.main()
