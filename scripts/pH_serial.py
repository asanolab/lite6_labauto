#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import Float32
import serial
import time


class PHSensorNode:
    def __init__(self):
        rospy.init_node("ph_sensor_node")

        # parameter settings
        self.port = rospy.get_param("~port", "/dev/ttyACM2")
        self.baud = int(rospy.get_param("~baud", 9600))
        self.timeout = float(rospy.get_param("~timeout", 1.0))  # time out of serial reading
        self.publish_voltage = bool(rospy.get_param("~publish_voltage", True))

        self.pub_ph = rospy.Publisher("/pH_sensor/pH_value", Float32, queue_size=10)
        self.pub_v = None
        if self.publish_voltage:
            self.pub_v = rospy.Publisher("/pH_sensor/voltage", Float32, queue_size=10)

        self.ser = None
        self._connect_serial()
        self.k = 12.917919
        self.b = -17.832006

    def _connect_serial(self):
        while not rospy.is_shutdown():
            try:
                self.ser = serial.Serial(self.port, self.baud, timeout=self.timeout)
                time.sleep(0.3)  # to connect stably
                self.ser.reset_input_buffer()  # clean buffer data
                rospy.loginfo(f"[ph_sensor_node] Connected to {self.port} @ {self.baud} baud.")
                return
            except serial.SerialException as e:
                rospy.logwarn(f"[ph_sensor_node] Open {self.port} failed: {e}. Retry in 2s...")
                time.sleep(2.0)

    def _voltage_to_ph(self, v):
        # function between pH and voltage: pH = -12.99 * 电压 - 18.6
        ph = self.k * v + self.b
        return round(ph, 2)

    def spin(self):
        while not rospy.is_shutdown():
            if self.ser is None or not self.ser.is_open:
                self._connect_serial()
                continue
            try:
                raw = self.ser.readline()  # read data until '\n'
                if not raw:
                    continue
                s = raw.decode(errors="ignore").strip()  # delete \r \n blank
                try:
                    v = float(s)  # for digital data # serial.println(voltage)
                except ValueError:
                    # if there is non-digit data
                    rospy.logdebug(f"[ph_sensor_node] skip non-numeric line: {s!r}")
                    continue
                rospy.loginfo(f"current pH voltage: {v}")
                ph = self._voltage_to_ph(v)
                rospy.loginfo(f"Current pH value: {ph:.2f}")
                self.pub_ph.publish(Float32(data=ph))
                if self.pub_v:
                    self.pub_v.publish(Float32(data=v))

                rospy.logdebug(f"[ph_sensor_node] V={v:.4f} -> pH={ph:.2f}")

            except (serial.SerialException, OSError) as e:
                rospy.logwarn(f"[ph_sensor_node] Serial error: {e}. Reconnecting...")
                try:
                    if self.ser:
                        self.ser.close()
                except Exception:
                    pass
                self.ser = None
                time.sleep(1.0)


if __name__ == "__main__":
    try:
        node = PHSensorNode()
        node.spin()
    except rospy.ROSInterruptException:
        pass
