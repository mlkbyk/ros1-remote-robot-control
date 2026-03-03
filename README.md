ROS1 Remote Robot Control & AI-Powered Adaptive Circular SLAM

[TR] Bu depo, standart Agilex Limo robot altyapısı üzerine eklenmiş; AI destekli dairesel tarama algoritması, gelişmiş Action-Server mimarisi ve Qt tabanlı kontrol arayüzünü içeren özel bir geliştirme katmanıdır.

[EN] This repository is a custom development layer built on top of the standard Agilex Limo robot infrastructure; featuring an AI-powered adaptive circular scanning algorithm, an advanced Action-Server architecture, and a Qt-based control interface.
🇹🇷 Türkçe Dokümantasyon
🚀 Genel Bakış

Bu proje, robotun algı yeteneklerini optimize etmek için yapay zeka ile geliştirilmiş dairesel hareket stratejilerini (ACS) ve sistemi bir API gibi terminalden veya GUI üzerinden yönetmeyi sağlayan araçları içerir.
📦 Paket Yapısı

    adaptive_circular_slam: Yapay zeka destekli dairesel yörünge üretimi ve skorlama (AI Logic).

    robot_control_app: Qt5 tabanlı görsel arayüz. Görüntü aktarımı ve veri kaydı (rosbag).

    motion_server_pkg: Komutları işleyen Action Server (Merkezi Kontrolör).

    motion_client_pkg: Terminalden hızlı komut (CLI) gönderim aracı.

    my_robot_msgs: Projeye özel Action ve Mesaj tanımları.

🛠 Kurulum ve Derleme
Bash

# Bağımlılıkları yükleyin
sudo apt-get update
sudo apt-get install -y qtbase5-dev libopencv-dev ros-noetic-cv-bridge ros-noetic-image-transport

# Çalışma alanını derleyin
cd ~/msb_agilex_ws
catkin_make
source devel/setup.bash

🕹 Kullanım

1. Tüm Sistemi Başlatma (Server + GUI):
Bash

roslaunch motion_client_pkg system_local.launch

2. CLI (Terminalden Kontrol):

    Dur: rosrun motion_client_pkg motion_cli stop

    Hız: rosrun motion_client_pkg motion_cli vel --lin 0.1 --ang 0.3

    Hassas Dönüş: rosrun motion_client_pkg motion_cli turn --deg 90

3. AI Dairesel Tarama (ACS):
Bash

roslaunch adaptive_circular_slam acs_real.launch

🇺🇸 English Documentation
🚀 Overview

This project introduces AI-enhanced circular movement strategies (ACS) to optimize the robot's perception and mapping quality. It also provides a robust "API-like" CLI and a Qt-based GUI for comprehensive robot management.
📦 Package Descriptions

    adaptive_circular_slam: AI-powered trajectory generation and candidate scoring logic.

    robot_control_app: Qt5 GUI for motion control, sensor visualization, and rosbag management.

    motion_server_pkg: Central Action Server that bridges high-level goals to hardware commands.

    motion_client_pkg: Command Line Interface (CLI) for rapid terminal-based control.

    my_robot_msgs: Custom Action and Message definitions for the stack.

🛠 Installation & Build
Bash

# Install dependencies
sudo apt-get update
sudo apt-get install -y qtbase5-dev libopencv-dev ros-noetic-cv-bridge ros-noetic-image-transport

# Build the workspace
cd ~/msb_agilex_ws
catkin_make
source devel/setup.bash

🕹 Usage Guide

1. Launching the Full Stack (Server + GUI):
Bash

roslaunch motion_client_pkg system_local.launch

2. CLI Usage (API-like Control):

    Stop: rosrun motion_client_pkg motion_cli stop

    Velocity: rosrun motion_client_pkg motion_cli vel --lin 0.1 --ang 0.3

    Precision Drive: rosrun motion_client_pkg motion_cli drive --m 1.0

3. Running AI Adaptive Circular SLAM:
Bash

# For simulation
roslaunch adaptive_circular_slam acs_sim.launch
# For real robot
roslaunch adaptive_circular_slam acs_real.launch

🔧 Technical Details / Teknik Detaylar

    Target OS: Ubuntu 20.04 (ROS Noetic)

    AI Method: Trajectory scoring via circle_scoring.cpp and dynamic path generation.

    Interface: ActionLib for non-blocking motion execution.
