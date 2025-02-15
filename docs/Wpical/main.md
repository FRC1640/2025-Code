# WPIcal Guide – Calibrating AprilTags Like a Pro

WPIcal is a cross-platform tool designed to **calibrate the position and orientation** of AprilTags for the **FIRST Robotics Competition**.

---

## Steps to Use WPIcal

### 1. Launch WPIcal
- Open **Visual Studio Code**.
- Press `Ctrl+Shift+P`, type **WPILib**, or click the **WPILib logo** in the top-right corner.
- Select **"Start Tool"**, then choose **"WPIcal"**.

### 2. Calibrate the Camera
- **Use the same camera** intended for field use.
- **Capture videos** of a **"pinned" AprilTag** from multiple angles.
- **Store all calibration videos** in a **dedicated directory**.

### 3. Field Calibration
- Select the **field map JSON file** within WPIcal.
- Choose the **directory containing your calibration videos**.
- Designate the **pinned tag** used during video capture.

### 4. Review Calibration Results
- **After processing**, WPIcal will display **differences between reference tags and calibrated tags** in **meters and degrees**.
- **Verify** that the calibrated values **match expected positions and orientations**.

---

## Key Terms

- **Focused Tag** – The tag WPIcal is calculating the **position error** of.
- **Reference Tag** – The tag used as a **comparison point** for the Focused Tag.


For **detailed documentation** and **troubleshooting**, check out the **official WPIcal docs**.
