# Jetson-PC-Arduino-Communication-via-ROS-2
Jetson-PC-Arduino Communication via ROS 2

This repository demonstrates a distributed ROS 2 system that connects a Jetson Nano, a PC, and an Arduino. It enables advanced sensor/control logic running on Jetson or PC, and delegates real-world actuation (e.g., servos, motors) to the Arduino.
🔧 Key Features

    Multi-device communication using ROS 2 over LAN.

    Jetson Nano or PC runs sensor processing (e.g., camera, hand tracking).

    Arduino controls hardware (e.g., servo motors) via serial, commanded by ROS 2 topics.

    Works with GUI and computer vision nodes.

    Modular structure to extend with sensors (IMU, encoders, etc.).

📡 Architecture

             ┌──────────────┐
             │   PC / Jetson│
             │  (ROS 2 Node)│
             └──────┬───────┘
                    │ (ROS 2 Topic: /servo_angle)
             ┌──────▼───────┐
             │   WiFi/LAN   │
             └──────┬───────┘
                    │
             ┌──────▼───────┐
             │   Arduino    │
             │ (Serial Port)│
             └──────────────┘

🧠 Use Cases

    PC or Jetson does:

        Hand gesture recognition via OpenCV + MediaPipe.

        GUI control and visualization.

        Sensor data fusion.

    Arduino does:

        Servo actuation.

        Motor driver interface.

        GPIO control.

📁 Suggested Repo File Structure

jetson-pc-arduino-comm/
├── pc_gui_slider_node.py         # GUI from PC to Arduino
├── jetson_hand_tracking_node.py  # Jetson sends angle based on hand gesture
├── servo_node.py                 # Arduino interface (same as before)
├── launch/
│   └── network_launch.py         # ROS 2 launch config across devices
├── README.md
└── requirements.txt

⚙️ How It Works

    Jetson/PC runs ROS 2 node that processes input (GUI or hand tracking).

    Angle is published to /servo_angle topic.

    Another device (Jetson/PC) runs servo_node.py, which:

        Subscribes to /servo_angle

        Sends data to Arduino via /dev/ttyUSB0

🧪 How to Run

    Network setup: Make sure Jetson and PC are on the same LAN.

    Run ROS 2 nodes on both devices:

    # On Jetson or PC (Publisher)
    ros2 run your_package jetson_hand_tracking_node

    # On another PC or Jetson (Subscriber)
    ros2 run your_package servo_node

    On Arduino, use this sketch:

#include <Servo.h>
Servo myServo;

void setup() {
  Serial.begin(9600);
  myServo.attach(9);
}

void loop() {
  if (Serial.available()) {
    int angle = Serial.parseInt();
    if (angle >= 0 && angle <= 180) {
      myServo.write(angle);
    }
  }
}

🌐 Tips for Multi-Machine ROS 2

    Set environment variables:

export ROS_DOMAIN_ID=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DISCOVERY_SERVER=IP_OF_JETSON:11811

