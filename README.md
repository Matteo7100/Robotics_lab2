# Robotics Lab Assignment 2 - Implementation of a Human Augmentation Device

Implementation of a human augmentation device, that can be attached to a human body to assist with a 1 dof linear motion.  

**Course**: Robotics, P2 2025-2026  
**Instructor**: joao.silva.sequeira@tecnico.ulisboa.pt  
**Due**: 16-1-2026, 23:59:59

## First Steps - Setup

### Prerequisites
- Ubuntu (or Linux-based system)
- Arduino IDE
- Python 3.x
- Processing IDE (for the digital twin)

### Installation and Setup

1. **Clone the repository:**
```bash
   git clone <repository-url>
   cd <repository-name>
```  
2. **Upload**: Open LAB2_arduino.ino and upload it to Arduino Uno.  
3. **Homing**: Upon startup, the system will automatically perform a Homing Sequence:  
      - Move Left until SW_LEFT is hit.  
      - Move Right until SW_RIGHT is hit to measure total travel steps.  
      - Move back to the calculated center.
4. **Gesture Execution**: Trigger a "button push" action based on gesture detection.

### Gesture execution
The "Push" command is activated by a distinct rotational gesture of the forearm. To trigger the sequence, the user rotates their wrist along the Roll axis until it exceeds a 90-degree threshold, holding the position briefly (120ms) to confirm the intent.

### Serial Commands
You can control the device manually by typing these characters into the Serial Monitor:

* `H` : **Re-Home** the device (Run calibration again).
* `P` : Trigger the **Button Push** sequence manually.
* `C` : **Calibrate** the Gyroscope bias (keep sensor still).
* `S` : **Start** CSV data streaming.
* `s` : **Stop** CSV data streaming.
* `?` : Print help menu

## Digital Twin Visualization





