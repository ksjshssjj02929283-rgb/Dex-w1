"""离线验证发送调度与 Qt 兼容记录；不导入 ROS、不连接机器人。"""

import io
import json
from types import SimpleNamespace
import unittest
from unittest.mock import patch

import numpy as np

import stream_arm_trajectory as stream


class StreamArmTrajectoryTest(unittest.TestCase):
    def run_schedule(self, period, first_publish_cost=0.0):
        clock = [0.0]
        sent = []
        records = []
        ros = SimpleNamespace(ok=lambda: True)  # 发送线程不能调用 spin_once。
        node = SimpleNamespace(get_clock=lambda: SimpleNamespace(
            now=lambda: SimpleNamespace(to_msg=lambda: clock[0])))
        names = [f"LEFT_J{i}" for i in range(1, 8)]
        positions = np.arange(35, dtype=float).reshape(5, 7)

        def publish(message):
            sent.append((clock[0], message))
            if len(sent) == 1:
                clock[0] += first_publish_cost

        def sleep(duration):
            self.assertGreater(duration, 0)
            self.assertLessEqual(duration, 0.001)
            clock[0] += duration

        with patch.object(stream.time, "monotonic", lambda: clock[0]), \
                patch.object(stream.time, "sleep", sleep):
            completed = stream.publish_trajectory(
                node, SimpleNamespace(publish=publish),
                lambda: SimpleNamespace(header=SimpleNamespace()),
                positions, names, period, ros,
                on_point=lambda: records.append(clock[0]),
            )
        self.assertTrue(completed)
        self.assertEqual(len(records), len(positions))
        for (_, message), expected in zip(sent, positions):
            self.assertEqual(message.name, names)
            np.testing.assert_array_equal(message.position, expected)
        return [when for when, _ in sent]

    def test_both_speeds_use_fixed_deadlines(self):
        for period in (0.01, 0.02):
            with self.subTest(period=period):
                np.testing.assert_allclose(self.run_schedule(period), np.arange(5) * period)

    def test_overrun_catches_up_like_qt_without_resetting_clock(self):
        np.testing.assert_allclose(self.run_schedule(0.01, 0.035), [0, 0.035, 0.035, 0.035, 0.04])

    def test_recording_is_buffered_and_saves_only_captured_rows(self):
        for arm in ("LEFT", "RIGHT"):
            with self.subTest(arm=arm):
                names = [f"{arm}_J{i}" for i in range(1, 8)]
                feedback = stream.ArmFeedback(names)
                state = {
                    "status": "Idle", "timestamp": 1_700_000_000_000_000_000,
                    "joint_name": names[::-1],
                    "joint_position": list(range(7))[::-1],
                    "joint_torque": list(range(10, 17))[::-1],
                    "joint_torque_sensor": list(range(20, 27))[::-1],
                }
                feedback.update(SimpleNamespace(data=json.dumps(state)))
                file = io.StringIO()
                recorder = stream.QtRecorder(file, feedback, capacity=6000)
                state["timestamp"] += 20_000_000
                feedback.update(SimpleNamespace(data=json.dumps(state)))
                recorder.record()
                self.assertEqual(file.getvalue(), "")
                recorder.save()
                data = np.loadtxt(io.StringIO(file.getvalue()), ndmin=2)
                self.assertEqual(data.shape, (1, 22))
                np.testing.assert_allclose(data[0], [.02, *range(7), *range(10, 17), *range(20, 27)])

    def test_snapshot_keeps_fields_from_one_feedback(self):
        names = [f"LEFT_J{i}" for i in range(1, 8)]
        feedback = stream.ArmFeedback(names)
        state = {"status": "Idle", "timestamp": 100, "joint_name": names,
                 "joint_position": [1.] * 7, "joint_torque": [2.] * 7}
        feedback.update(SimpleNamespace(data=json.dumps(state)))
        previous = feedback.snapshot()
        state.update(timestamp=200, joint_position=[3.] * 7, joint_torque=[4.] * 7)
        feedback.update(SimpleNamespace(data=json.dumps(state)))
        self.assertEqual(previous[1], 100)
        np.testing.assert_array_equal(previous[2], [1.] * 7)
        np.testing.assert_array_equal(previous[3], [2.] * 7)
        np.testing.assert_array_equal(previous[4], [0.] * 7)
        self.assertEqual(feedback.snapshot()[1], 200)

    def test_stale_or_error_feedback_stops_capture(self):
        feedback = stream.ArmFeedback([f"LEFT_J{i}" for i in range(1, 8)])
        state = {"status": "Error", "timestamp": 0, "joint_name": feedback.names,
                 "joint_position": [0.] * 7}
        feedback.update(SimpleNamespace(data=json.dumps(state)))
        with self.assertRaises(RuntimeError):
            feedback.snapshot()
        state["status"] = "Idle"
        feedback.update(SimpleNamespace(data=json.dumps(state)))
        with patch.object(stream.time, "monotonic", lambda: feedback.received_at + 0.6):
            with self.assertRaises(RuntimeError):
                feedback.snapshot()


if __name__ == "__main__":
    unittest.main()
