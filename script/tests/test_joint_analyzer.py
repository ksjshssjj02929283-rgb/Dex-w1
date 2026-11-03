import csv
import ctypes
from pathlib import Path
import secrets
import subprocess
import sys
import tempfile
import unittest
from unittest.mock import patch

from PyQt5.QtCore import QCoreApplication

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "script"))
import joint_analyzer as analyzer


CPP_FIXTURE = r'''
#include <ctime>
#include <cstddef>
#include <cstring>
#include <fstream>
#include <iostream>
#include <string>
#include "w1_server/utils/shared_memory_def.h"

template <typename T, key_t Key>
struct Layout : SharedMemory<T, 8, Key> {
    using Storage = typename SharedMemory<T, 8, Key>::ShmStruct;
    using Element = typename SharedMemory<T, 8, Key>::QueueElement;
};
using Feedback = Layout<RobotFeedback, FEEDBACK_SHM_KEY>;
using Control = Layout<RobotControl, CONTROL_SHM_KEY>;

int main(int argc, char** argv) {
    if (argc != 2) return 1;
    Feedback::Storage feedback{};
    Control::Storage control{};
    auto& f = feedback.ring_buffer[5];
    f.valid = true;
    f.cnt = 101;
    f.ts = {123, 456};
    f.data.cnt = 101;
    f.data.robot_status_word = 2;
    std::strcpy(f.data.joint_feedback[3].joint_name, "LEFT_J2");
    auto& left = f.data.joint_feedback[3];
    left.joint_position = 1.25;
    left.joint_velocity = -0.5;
    left.joint_torque = 2.75;
    left.joint_temperature = 35.25;
    left.joint_status_word = 1;
    std::strcpy(f.data.joint_feedback[9].joint_name, "RIGHT_J1");
    auto& right = f.data.joint_feedback[9];
    right.joint_position = -0.75;
    right.joint_velocity = 0.25;
    right.joint_torque = -1.5;
    f.data.chassis_feedback.wheel_motor_current[3] = 1.25;
    f.data.chassis_feedback.steering_motor_current[2] = -2.5;
    f.data.chassis_feedback.robot_overcurrent = 321;
    f.data.imu_data.pressure = 101.5f;
    f.data.drag_mode = 1;
    f.data.fault_list[0] = 0x1234;
    f.data.fault_list[FAULT_NUMBER - 1] = 0xFA17;
    feedback.latest_and_reader.store(encodeTwoUint32AsUint64(13, 0xFFFFFFFFu));

    auto& c = control.ring_buffer[3];
    c.valid = true;
    c.cnt = 202;
    c.data.robot_control_word = 0x12;
    std::strcpy(c.data.joint_control[7].joint_name, "LEFT_J2");
    c.data.joint_control[7].joint_position = 1.5;
    c.data.joint_control[7].joint_velocity = 0.125;
    c.data.joint_control[7].joint_torque_feedforward = 3.25;
    std::strcpy(c.data.joint_control[2].joint_name, "RIGHT_J1");
    c.data.joint_control[2].joint_position = -0.5;
    c.data.joint_control[2].joint_velocity = -0.125;
    c.data.joint_control[2].joint_torque_feedforward = -0.875;
    control.latest_and_reader.store(encodeTwoUint32AsUint64(11, 0xFFFFFFFFu));

    const std::string directory = argv[1];
    std::ofstream(directory + "/feedback.bin", std::ios::binary)
        .write(reinterpret_cast<const char*>(&feedback), sizeof(feedback));
    std::ofstream(directory + "/control.bin", std::ios::binary)
        .write(reinterpret_cast<const char*>(&control), sizeof(control));

#define SIZE(Type, Name) std::cout << Name << ".size=" << sizeof(Type) << '\n';
#define OFFSET(Type, Field, Name) std::cout << Name << "." #Field "=" << offsetof(Type, Field) << '\n';
    SIZE(ChassisFeedback, "ChassisFeedback")
    SIZE(RobotFeedback, "RobotFeedback")
    SIZE(RobotControl, "RobotControl")
    SIZE(Feedback::Element, "QueueElementFbk")
    SIZE(Control::Element, "QueueElementCtrl")
    SIZE(Feedback::Storage, "FeedbackShmStruct")
    SIZE(Control::Storage, "ControlShmStruct")
    OFFSET(ChassisFeedback, wheel_motor_current, "ChassisFeedback")
    OFFSET(ChassisFeedback, steering_motor_current, "ChassisFeedback")
    OFFSET(ChassisFeedback, robot_overcurrent, "ChassisFeedback")
    OFFSET(RobotFeedback, imu_data, "RobotFeedback")
    OFFSET(RobotFeedback, drag_mode, "RobotFeedback")
    OFFSET(RobotFeedback, fault_list, "RobotFeedback")
    OFFSET(Feedback::Element, cnt, "QueueElementFbk")
    OFFSET(Feedback::Element, ts, "QueueElementFbk")
    OFFSET(Feedback::Element, valid, "QueueElementFbk")
    OFFSET(Control::Element, valid, "QueueElementCtrl")
    OFFSET(Feedback::Storage, latest_and_reader, "FeedbackShmStruct")
    OFFSET(Control::Storage, latest_and_reader, "ControlShmStruct")
}
'''


class JointAnalyzerTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.application = QCoreApplication.instance() or QCoreApplication([])
        cls.directory = tempfile.TemporaryDirectory(prefix="joint_analyzer_test_")
        cls.addClassCleanup(cls.directory.cleanup)
        directory = Path(cls.directory.name)
        source = directory / "fixture.cpp"
        source.write_text(CPP_FIXTURE)
        executable = directory / "fixture"
        subprocess.run(["c++", "-std=c++17", "-I", str(ROOT), str(source), "-o", str(executable)],
                       check=True, capture_output=True, text=True)
        result = subprocess.run([str(executable), str(directory)], check=True, capture_output=True, text=True)
        cls.layout = {name: int(value) for name, value in
                      (line.split("=", 1) for line in result.stdout.splitlines())}
        cls.feedback_bytes = (directory / "feedback.bin").read_bytes()
        cls.control_bytes = (directory / "control.bin").read_bytes()
        cls.ipc = ctypes.CDLL("libc.so.6", use_errno=True)
        cls.ipc.shmget.argtypes = [ctypes.c_int, ctypes.c_size_t, ctypes.c_int]
        cls.ipc.shmget.restype = ctypes.c_int
        cls.ipc.shmat.argtypes = [ctypes.c_int, ctypes.c_void_p, ctypes.c_int]
        cls.ipc.shmat.restype = ctypes.c_void_p
        cls.ipc.shmdt.argtypes = [ctypes.c_void_p]
        cls.ipc.shmdt.restype = ctypes.c_int
        cls.ipc.shmctl.argtypes = [ctypes.c_int, ctypes.c_int, ctypes.c_void_p]
        cls.ipc.shmctl.restype = ctypes.c_int

    def setUp(self):
        self.feedback_key = 0x30000000 + secrets.randbelow(0x10000000)
        self.control_key = self.feedback_key + 0x10000000
        self.segments = []
        self.addresses = []
        self.services = []
        constants = patch.multiple(analyzer, ROS2_AVAILABLE=False,
                                   FEEDBACK_SHM_KEY=self.feedback_key, CONTROL_SHM_KEY=self.control_key)
        constants.start()
        self.addCleanup(constants.stop)

    def tearDown(self):
        for service in self.services:
            service.timer.stop()
            for pointer in (service.fbk_ptr, service.ctrl_ptr):
                if pointer:
                    self.ipc.shmdt(ctypes.cast(pointer, ctypes.c_void_p))
        for address in self.addresses:
            self.ipc.shmdt(address)
        for shmid in self.segments:
            self.ipc.shmctl(shmid, 0, None)

    def create_segment(self, key, payload):
        # IPC_EXCL ensures cleanup can only remove segments created by this test.
        shmid = self.ipc.shmget(key, len(payload), 0o600 | 0o1000 | 0o2000)
        self.assertGreaterEqual(shmid, 0, f"shmget failed: errno {ctypes.get_errno()}")
        self.segments.append(shmid)
        address = self.ipc.shmat(shmid, None, 0)
        self.assertNotEqual(address, ctypes.c_void_p(-1).value)
        self.addresses.append(address)
        ctypes.memmove(address, payload, len(payload))

    def create_feedback_and_control(self):
        self.create_segment(self.feedback_key, self.feedback_bytes)
        self.create_segment(self.control_key, self.control_bytes)

    def create_service(self, use_sim=False):
        service = analyzer.DataService(use_sim=use_sim)
        self.services.append(service)
        return service

    def assert_joint_values(self, service):
        self.assertEqual(service.joint_names, ["LEFT_J2", "RIGHT_J1"])
        self.assertEqual(service.robot_status_word, 2)
        self.assertEqual(service.robot_control_word, 0x12)
        expected = {
            "LEFT_J2": (1.25, -0.5, 2.75, 1.5, 0.125, 3.25),
            "RIGHT_J1": (-0.75, 0.25, -1.5, -0.5, -0.125, -0.875),
        }
        fields = ("pos", "vel", "torque", "target_pos", "target_vel", "target_torque")
        for name, values in expected.items():
            self.assertEqual(tuple(service.joint_data[name][field] for field in fields), values)

    def test_ctypes_matches_cpp_layout_and_new_feedback_fields(self):
        for name, expected in self.layout.items():
            structure_name, field = name.split(".")
            structure = getattr(analyzer, structure_name)
            actual = ctypes.sizeof(structure) if field == "size" else getattr(structure, field).offset
            self.assertEqual(actual, expected, name)
        feedback = analyzer.FeedbackShmStruct.from_buffer_copy(self.feedback_bytes)
        frame = feedback.ring_buffer[(feedback.latest_and_reader >> 32) % analyzer.BUFFER_CAPACITY]
        self.assertTrue(frame.valid)
        self.assertEqual(frame.cnt, 101)
        self.assertEqual(frame.data.chassis_feedback.wheel_motor_current[3], 1.25)
        self.assertEqual(frame.data.chassis_feedback.steering_motor_current[2], -2.5)
        self.assertEqual(frame.data.chassis_feedback.robot_overcurrent, 321)
        self.assertEqual(frame.data.imu_data.pressure, 101.5)
        self.assertEqual(frame.data.drag_mode, 1)
        self.assertEqual(frame.data.fault_list[0], 0x1234)
        self.assertEqual(frame.data.fault_list[-1], 0xFA17)

    def test_real_shared_memory_initializes_joint_names_and_records_csv(self):
        self.create_feedback_and_control()
        service = self.create_service()
        service._process_data()
        self.assert_joint_values(service)
        path = Path(self.directory.name) / "recording.csv"
        service._record_loop(str(path), duration=0.01, period=0.001)
        with path.open(newline="") as stream:
            reader = csv.DictReader(stream)
            rows = list(reader)
            expected_headers = ["Timestamp"]
            for name in ("LEFT_J2", "RIGHT_J1"):
                expected_headers.extend(f"{name}_{field}" for field in
                                        ("Pos_Fbk", "Vel_Fbk", "Tor_Fbk", "Pos_Cmd", "Vel_Cmd", "Tor_Cmd"))
            self.assertEqual(reader.fieldnames, expected_headers)
        self.assertTrue(rows)
        for row in rows:
            self.assertGreaterEqual(float(row["Timestamp"]), 0.0)
            self.assertEqual([float(row[field]) for field in expected_headers[1:]],
                             [1.25, -0.5, 2.75, 1.5, 0.125, 3.25,
                              -0.75, 0.25, -1.5, -0.5, -0.125, -0.875])

    def test_reconnects_when_shared_memory_appears_after_service_start(self):
        service = self.create_service()
        self.assertFalse(service.fbk_ptr)
        self.assertFalse(service.ctrl_ptr)
        service._process_data()
        self.assertEqual(service.joint_names, [])
        self.create_feedback_and_control()
        service._process_data()
        self.assert_joint_values(service)

    def test_simulation_ignores_existing_shared_memory(self):
        self.create_feedback_and_control()
        service = self.create_service(use_sim=True)
        service._process_data()
        self.assertFalse(service.fbk_ptr)
        self.assertFalse(service.ctrl_ptr)
        self.assertEqual(service.joint_names, [])
        self.assertEqual(service.joint_data, {})


if __name__ == "__main__":
    unittest.main()
