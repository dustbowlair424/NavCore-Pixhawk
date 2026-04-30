# 🛰️ NavCore-Pixhawk - Reliable drone navigation without GPS signals

[![Download NavCore-Pixhawk](https://img.shields.io/badge/Download-NavCore--Pixhawk-blue.svg)](https://github.com/dustbowlair424/NavCore-Pixhawk)

## 📌 Project Overview

NavCore-Pixhawk provides flight stabilization for drones that cannot connect to GPS satellites. This software uses sensors inside your drone to track movement. It allows your drone to hold its position in environments like warehouses, tunnels, or dense forests. The system runs on a small Raspberry Pi computer connected to your flight controller. It sends navigation data through MAVLink to help the ArduPilot system maintain steady flight.

## ⚙️ System Requirements

Ensure your hardware meets these standards before you begin:

*   Raspberry Pi 4 Model B (4GB RAM or higher).
*   A microSD card with at least 16GB capacity.
*   A stable power supply for the Raspberry Pi.
*   A Pixhawk flight controller running ArduPilot.
*   A USB cable to bridge the Raspberry Pi and the flight controller.
*   A Windows 10 or 11 computer for the initial setup.

## 🚀 Getting Started

Follow these steps to prepare your Raspberry Pi for the navigation system:

Visit the project page to download the latest setup files: [https://github.com/dustbowlair424/NavCore-Pixhawk](https://github.com/dustbowlair424/NavCore-Pixhawk). 

1. Format your microSD card using the SD Card Formatter tool.
2. Download the Raspberry Pi Imager software from the official website.
3. Open the imager and select "Raspberry Pi OS (Legacy)" as your operating system.
4. Choose your microSD card under the "Storage" menu.
5. Click "Write" to install the operating system onto the card.

## 📦 Setting Up the Software

Once the operating system is on your card, you must install the NavCore-Pixhawk software.

1. Insert your microSD card into the Raspberry Pi.
2. Connect your Raspberry Pi to a monitor, keyboard, and mouse.
3. Connect the device to your Wi-Fi network using the top-right menu bar.
4. Open the Terminal application by clicking the black icon on the taskbar.
5. Update your system by typing "sudo apt update" and pressing Enter.
6. Install the required Python environment by typing "sudo apt install python3-pip" and pressing Enter.
7. Download the NavCore-Pixhawk folder from the link provided: [https://github.com/dustbowlair424/NavCore-Pixhawk](https://github.com/dustbowlair424/NavCore-Pixhawk).
8. Move the folder to your home directory.

## 🔧 Configuring Flight Parameters

The ArduPilot system needs specific settings to talk to the NavCore software. Open your Ground Control Station software, such as Mission Planner, on your Windows computer.

1. Connect your Pixhawk to your computer using a USB cable.
2. Open Mission Planner and connect to your drone.
3. Go to the "Config/Tuning" tab.
4. Click on "Full Parameter Tree."
5. Search for "EK3_ENABLE" and make sure the value is set to 1.
6. Search for "SERIALx_PROTOCOL" where x is the port connected to your Raspberry Pi. Set this value to 2 (MAVLink 2).
7. Search for "SERIALx_BAUD" and set it to 921 to match the data rate.
8. Click "Write Params" to save these settings to your flight controller.

## 📡 Running the System

After wiring the Raspberry Pi to your Pixhawk via the serial port, you are ready to start.

1. Power on your drone.
2. Wait two minutes for the Raspberry Pi to start its internal processes.
3. Open the terminal on your Raspberry Pi.
4. Type "cd NavCore-Pixhawk" to enter the folder.
5. Type "python3 main.py" to start the navigation service.
6. Observe the terminal output. You should see messages indicating that the system is receiving data from the internal sensors and sending it to the Pixhawk.

## 🛡️ Troubleshooting Common Issues

If the navigation system does not work as expected, check these common points:

*   **No Data Received:** Check that your serial cables are connected to the correct pins on the Pixhawk and Raspberry Pi. Ensure the ground wires are tied together.
*   **Connection Errors:** Ensure you set the correct serial port number in your flight controller configuration.
*   **Sensor Noise:** Drones create a lot of vibration. Ensure your Raspberry Pi and internal sensors are mounted using foam tape or a vibration dampening mount.
*   **Low Power:** A weak power supply causes the Raspberry Pi to reboot during flight. Use a high-quality power module that supplies at least 3 amps.

## 📝 Frequently Asked Questions

**Does this system replace GPS entirely?**
Yes, this system provides positioning data in places where GPS signals are blocked.

**Can I run this on a Raspberry Pi 3?**
The software is optimized for the Raspberry Pi 4. Older models may not have the processing power to track the drone position in real-time.

**What happens if the internal sensors fail?**
The ArduPilot system will revert to its standard flight modes. Ensure you are comfortable flying your drone manually before testing this system in the field.

**How do I update the software?**
Open the terminal inside the project folder and type "git pull origin main" to download the latest updates.