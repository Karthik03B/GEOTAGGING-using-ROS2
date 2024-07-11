#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import RPi.GPIO as GPIO
import time
from std_msgs.msg import Int32

PWM_PIN = 17  # GPIO pin connected to the PWM signal
CYCLE_TIME = 0.2  # Time between measurements in seconds

class PWMReaderNode(Node):
    def __init__(self):
        super().__init__('pwm_reader_node')
        self.trigger = self.create_publisher(Int32, 'trigger_pub', 10)
        self.setup_gpio()
        self.timer = self.create_timer(CYCLE_TIME, self.read_pwm)

    def setup_gpio(self):
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(PWM_PIN, GPIO.IN)

    def read_pwm(self):
        start_time = time.time()
        while GPIO.input(PWM_PIN) == GPIO.LOW:
            pass
        pulse_start = time.time()

        while GPIO.input(PWM_PIN) == GPIO.HIGH:
            pass
        pulse_end = time.time()

        pulse_duration = pulse_end - pulse_start
        if pulse_duration == 0:
            #self.get_logger().warn("Pulse duration is zero")
            return
        
        frequency = 1 / pulse_duration
        duty_cycle = (pulse_duration / (pulse_duration + (time.time() - start_time))) * 100
        
        if 495 < frequency < 505:
            msg = Int32()
            msg.data = 1
            self.trigger.publish(msg)

        #self.get_logger().info(f"Frequency: {frequency:.2f} Hz, Duty Cycle: {duty_cycle:.2f} %")

def main(args=None):
    rclpy.init(args=args)
    node = PWMReaderNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        GPIO.cleanup()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
