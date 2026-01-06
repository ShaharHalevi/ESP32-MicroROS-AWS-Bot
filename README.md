Autonomous IoT Inventory Robot
==============================

**An End-to-End Industry 4.0 Solution**

\[Image Placeholder: Robot Scanning in Warehouse\]

### **The Vision**

This project bridges the gap between physical robotics and cloud intelligence. It is not just a robot; it is a fully autonomous agent capable of navigating dynamic environments, digitizing physical inventory, and syncing with the cloud in real-time. Designed as a Computer Engineering Final Project, it simulates a real-world logistics solution using cutting-edge technologies.

### **Core Technologies**

*   **Embedded Control:** ESP32-CAM & FreeRTOS
    
*   **Robotics Middleware:** Micro-ROS & ROS 2
    
*   **Cloud Backend:** AWS IoT Core, Lambda, DynamoDB
    
*   **Visualization:** Amazon Grafana
    

### **System Architecture: How It Works**

The system creates a seamless data pipeline from the warehouse floor to the manager's screen in milliseconds:

**1\. The Edge (The Robot)**
The robot is the eyes on the ground. Powered by an **ESP32-CAM**, it runs a dual-core architecture to handle image processing and motor control simultaneously. It operates as a native Micro-ROS node, autonomously navigating and decoding QR codes.

**2\. The Bridge (Local Gateway)**
To ensure security and protocol translation, a Dockerized Micro-ROS Agent bridges the robot's UDP signals to the ROS 2 network. A custom **Python Gateway** then encrypts the data using X.509 certificates and forwards it to the cloud.

**3\. The Cloud (AWS Backend)**
The data lands in **AWS IoT Core**, triggering a Serverless Lambda function. This function enriches the data with precise server-side timestamps and stores the inventory state in a NoSQL **DynamoDB** table.

**4\. The Insight (Dashboard)Grafana** visualizes the data instantly, showing live stock levels ("Apples: 50") and historical trends, enabling data-driven decision-making.

### **Engineering Highlights**

**Problem:** Mechanical Instability
**Solution:** The "Reverse Patrol" AlgorithmA common issue with differential drive robots is caster-wheel flutter when driving forward. I implemented a novel software solution that inverts the control logic, driving the robot "backwards" to drag the caster wheel for perfect stability.

**Problem:** Resource Constraints
**Solution:** FreeRTOS MultitaskingTo prevent the camera's heavy image processing from stalling the motors, the firmware pins the Vision Task to Core 1 and the Network/Control Task to Core 0, ensuring smooth, non-blocking operation.

### **Hardware Components**

*   **Controller:** ESP32-CAM (AI-Thinker)
    
*   **Drive System:** L298N H-Bridge with DC Gear Motors
    
*   **Power:** High-discharge 18650 Li-Ion Battery Pack (7.4V)
    
*   **Chassis:** Custom Acrylic Platform
    

\[Image Placeholder: Wiring Diagram / Circuit Board\]

### **See It In Action**

\[Link Placeholder: YouTube Demo Video\]_Watch the robot autonomously scan items and update the dashboard in real-time._

**Author:** Shahar Halevi_Computer Engineering Student_
