# Mobile-Robotics-Assessment
Main Assessment Files; Videos, Maps, parameters and Code


# 🤖 Mobile Robotics Assessment – TurtleBot4 Navigation Suite

Please Watch My Video Demonstration through the link below:
📹 **[Video Submission](https://www.play.mdx.ac.uk/media/Mobile+Robotics+Assessment+Video/1_k0p1y8lm)**  
📱 **[React Native App GitHub Repo](https://github.com/Shuaibu-oluwatunmise/TurtleBot4App)**

---

## 📦 About the Project

This is a ROS 2-based autonomous navigation project built around the TurtleBot4 platform. The system is capable of localizing, planning, and navigating autonomously to different locations using RViz tools.

In addition to the core functionality, this project includes **extra features** such as:
- 🔊 Voice-Controlled Navigation (Offline, using Vosk)
- 📱 A mobile app for remote monitoring and teleoperation

All scripts are organized in a single folder on GitHub for simplicity.

---

## 🚀 Getting Started

### 🔧 Prerequisites

- ROS 2 Jazzy (Ubuntu 24.04)
- TurtleBot4 (real or simulated)
- Working microphone (for voice control)
- Optional: Mobile phone with the custom React Native app

---

## 🗺️ Launching Autonomous Navigation

Open a terminal in your ROS 2 environment. First, clone this repo:

```bash
git clone <your-repo-url>
cd <project-folder>
```

Then, open **three tabs** in your terminal and run the following in each:

**🧭 Tab 1 – Localization:**
```bash
ros2 launch turtlebot4_navigation localization.launch.py map:=/home/ros/my_map-yaml.yaml params:=/home/ros/nav/localization.yaml
```

**📍 Tab 2 – Navigation:**
```bash
ros2 launch turtlebot4_navigation nav2.launch.py params_file:=/home/ros/nav/nav2.yaml
```

**🖥️ Tab 3 – RViz Visualization:**
```bash
ros2 launch turtlebot4_viz view_navigation.launch.py
```

> ⚠️ Make sure to replace the file paths with the actual locations of your `map.yaml`, `localization.yaml`, and `nav2.yaml` files.

These three commands will:
- Load your saved map
- Launch the Nav2 stack
- Start RViz for setting goals and observing navigation

Now, your robot is ready to navigate using **2D Pose Estimate** and **Nav2 Goal** tools in RViz!

---

## 🧪 Optional Scripts

Open a **fourth tab** and run any of these:

```bash
python3 path_to_script.py
```

### 🌀 `navtopose.py`
Sends the robot to a single predefined pose. Great for testing quick navigation or simple demos.

### ➰ `navthroughposes.py`
Sends the robot through multiple waypoints sequentially. Useful for delivery routes or patrolling.

### 🎯 `chooseposes.py`
Lets you choose from predefined poses interactively—ideal for dynamic but structured navigation.

---

## 🗣️ EXTRA FEATURE: Voice-Controlled Navigation

### Why Vosk?
I chose **Vosk** because it’s fully **offline**, light, and easy to integrate with ROS. This demo showcases how voice recognition can be used to navigate to specific rooms—making it a powerful, scalable interface for assistive robotics.

---

### 🧰 Set Up Voice Navigation

#### ✅ 1. Create Python Virtual Environment

```bash
sudo apt update
sudo apt install python3-venv python3-pip portaudio19-dev
python3 -m venv ~/turtlebot_voice_env
source ~/turtlebot_voice_env/bin/activate
pip install --upgrade pip
```

> ℹ️ ROS environments sometimes block microphone access. That's why we use a virtual environment here.

---

#### ✅ 2. Install Required Packages

```bash
pip install vosk pyaudio numpy pyyaml jinja2 setuptools typeguard
```

---

#### ✅ 3. Download and Extract Vosk Model

```bash
cd ~/Downloads
wget https://alphacephei.com/vosk/models/vosk-model-small-en-us-0.15.zip
unzip vosk-model-small-en-us-0.15.zip
```

> Keep the folder `vosk-model-small-en-us-0.15` in your project directory.

---

#### ✅ 4. (Optional) Test Your Microphone

```bash
python3 path_to_mic_test.py
```

---

#### ✅ 5. Launch the Navigation Stack (Again)

In case you haven't already, run the 3 commands from earlier to start localization, navigation, and RViz.

---

#### ✅ 6. Run the Voice Control Script

```bash
source ~/turtlebot_voice_env/bin/activate
python3 VoiceControlled.py
```

Now speak any of the following commands:

```
home
kitchen
dining
sitting room
exit
```

The robot will navigate to the corresponding location automatically using offline speech recognition! 🎉

---

## 📱 EXTRA FEATURE: Mobile Teleoperation App

I also developed a **React Native mobile app** that connects to the TurtleBot using **rosbridge** and WebSocket communication.

### 📲 Features

- 🔋 Battery Status and History – beautiful, real-time UI
- 🎮 Two Modes of Teleoperation:
  - **Joystick Mode**
  - **Button Pad Mode**
- 📡 Live telemetry and speed control
- 💅 Sleek, user-intuitive interface

> The app allows users to **drive the robot remotely** with ease, with full control and live feedback.

---

### 🔧 How it Works

The app uses:
- WebSocket communication via `rosbridge_suite`
- React Native frontend with TypeScript
- ROS messages for velocity and feedback

I'm currently working on:
- 🗺️ Map creation (live visualization while driving)
- 🧠 Autonomous mapping + vision
- 🖥️ Web dashboard + GUI triggers

> Mapping will be implemented using Python to receive `/map` data and React Canvas to draw it in real-time.

---

### ⚠️ Disclaimer

> This mobile app is **still under development**.

To run it:
- SSH into the TurtleBot or your ROS VM
- Install `rosbridge_suite` if not already installed
- Make sure WebSocket server is running
- (Optional) Add it to the robot’s startup script for automation

---

## 🌟 Future Work

- SLAM + automatic map saving
- Tap-to-navigate from phone
- Natural language processing for commands
- Camera integration for visual tasks (e.g. object detection)

---

## 📬 Feedback

Feel free to reach out or contribute ideas!  
Pull requests, feedback, and suggestions are highly welcome.

---
