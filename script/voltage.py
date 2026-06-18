# dexforce@jetson-orin-nx:~$ ethercat upload -p1 0x6079 0
# 0x0000c29d 49821
# 写一个python程序，不断运行这个命令，读取结果。其中49821就代表49.821V。
# 用ros2 消息发出

import subprocess

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

class VoltagePublisher(Node):
	def __init__(self) -> None:
		super().__init__("voltage_publisher")
		self.publisher_ = self.create_publisher(Float32, "voltage", 10)
		self.timer = self.create_timer(0.2, self.publish_voltage)
		self.cmd = ["ethercat", "upload", "-p2", "0x6079", "0"]

	def read_voltage(self) -> float | None:
		try:
			result = subprocess.run(
				self.cmd,
				check=True,
				capture_output=True,
				text=True,
			)
		except FileNotFoundError:
			self.get_logger().error("ethercat command not found.")
			return None
		except subprocess.CalledProcessError as exc:
			self.get_logger().error(f"ethercat command failed: {exc.stderr.strip()}")
			return None

		output = result.stdout.strip()
		if not output:
			self.get_logger().warn("ethercat returned empty output.")
			return None

		parts = output.split()
		if len(parts) < 2:
			self.get_logger().warn(f"unexpected ethercat output: {output}")
			return None

		try:
			milli_volt = int(parts[-1])
		except ValueError:
			self.get_logger().warn(f"failed to parse voltage from output: {output}")
			return None

		return milli_volt / 1000.0

	def publish_voltage(self) -> None:
		voltage = self.read_voltage()
		if voltage is None:
			return

		msg = Float32()
		msg.data = float(voltage)
		self.publisher_.publish(msg)
		self.get_logger().info(f"Published voltage: {msg.data:.3f} V")


def main(args=None) -> None:
	rclpy.init(args=args)
	node = VoltagePublisher()
	try:
		rclpy.spin(node)
	except KeyboardInterrupt:
		pass
	finally:
		node.destroy_node()
		rclpy.shutdown()


if __name__ == "__main__":
	main()


