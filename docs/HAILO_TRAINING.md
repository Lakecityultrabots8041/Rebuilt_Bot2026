# Hailo Fuel Detection - Training Guide

This covers how to train the Limelight 4's Hailo-8L chip to detect fuel on the field, from taking your first photo to the model running live on the robot.

Verify steps against current Limelight documentation at **docs.limelightvision.io** because this process can change between seasons.

---

## What You're Actually Doing

The Hailo chip runs a tiny neural network that looks at every camera frame and draws a box around anything it thinks is fuel. You train the network by showing it hundreds of photos of fuel labeled with boxes, and it learns the pattern. Once trained, the network runs entirely on the Limelight with no RoboRIO processing needed.

The Limelight then reports `tx`, `ty`, and `ta` for the detected object just like it does for AprilTags, which is what `FuelDetectionSubsystem` reads.

---

## Step 1 - Create a Roboflow Account

Go to **roboflow.com** and create a free account. Roboflow is where you collect, label, and train. FRC teams get free access.

Create a new project:
- Project type: **Object Detection**
- Annotation group: anything descriptive, e.g. `fuel`
- License: keep default

---

## Step 2 - Take Training Images

More images = more accurate model. Diverse images = more robust model.

### How to take images

Use your phone camera or a regular digital camera. You do not need the Limelight camera to take training images, a phone is faster and easier.

### What to photograph

| Scenario | Why it matters |
|---|---|
| Fuel on the carpet from 1-4 meters away | Main use case |
| Fuel at different angles (head-on, from the side) | Robot won't always approach straight |
| Fuel under match lighting (bright gym lights) | Matches won't be in a dark room |
| Fuel near the field wall or other yellow objects | Prevents false positives |
| Fuel partially blocked by another fuel or field element | Real-world occlusion |
| Fuel from low to the ground (intake camera angle) | Matches your actual camera position |
| Multiple fuel in one frame | Robot may see several at once |

### How many images

- **Minimum to get started:** 100 images
- **Good for competition:** 200-400 images
- **Excellent:** 500+

Spend one session at a practice field or with a mockup and you can get 200+ images in an hour.

### Tips for good images

- Take photos from roughly where the camera will be mounted on the robot, low and angled down
- Include blurry images intentionally (robot is moving)
- Include some images with NO fuel (teaches the model what "not fuel" looks like)
- Vary the background, carpet of different colors, field walls, other robots

---

## Step 3 - Upload to Roboflow

In your Roboflow project, click **Upload** and drag in all your photos. Roboflow accepts JPG, PNG, and most common formats.

After upload, Roboflow will ask how to split your dataset:
- **Train: 80%** (used to train the model)
- **Valid: 10%** (used to check accuracy during training)
- **Test: 10%** (used to verify final accuracy)

Click **Save and Continue**.

---

## Step 4 - Label Your Images

Click **Annotate** in Roboflow. For each image:

1. Draw a tight bounding box around each visible fuel
2. Assign the label `fuel` (or whatever you named your class, be consistent)
3. Skip images that are too blurry to label

Roboflow has **Smart Polygon** and **Label Assist** tools that auto-suggest boxes. Use them to speed this up.

If you have teammates, assign images between them. With 3-4 people you can label 300 images in an hour.

---

## Step 5 - Train the Model

Once labeled, click **Generate** in Roboflow to create a dataset version. Then click **Train with Roboflow** (or export for external training).

For Limelight 4 + Hailo, check Limelight's current documentation for the correct export format. As of the 2025-2026 season, the process is:

1. In Roboflow, go to **Export Dataset**
2. Select format: check docs.limelightvision.io for the current Hailo-compatible export format
3. Download the exported package
4. OR use Limelight's built-in training pipeline at their website (verify at docs.limelightvision.io)

Training takes 10-30 minutes on Roboflow's cloud, depending on dataset size.

---

## Step 6 - Deploy to Limelight

1. Open the Limelight web dashboard (robot must be connected, go to `limelight.local:5801` or the IP shown in Driver Station)
2. Go to **Pipelines**
3. Create a new pipeline, set type to **Neural Detector**
4. Upload your trained model file
5. Set this pipeline to index **1** (matching `VisionConstants.FUEL_DETECTOR_PIPELINE = 1`)

Pipeline 0 is left as AprilTag mode in case the camera is ever repurposed.

---

## Step 7 - Verify in the Dashboard

With the robot on and fuel in front of the camera:

1. Switch to pipeline 1 in the Limelight dashboard
2. You should see bounding boxes appear around fuel in the camera feed
3. Check the `tx`, `ty`, `ta` values update when fuel is visible

If boxes are missing or appearing on the wrong objects, go back and add more training images for those cases.

---

## Step 8 - Tune the Code Constants

Once the model is running, adjust these values in `VisionConstants.java` on the robot:

| Constant | What to tune |
|---|---|
| `FUEL_MIN_AREA` | Increase to ignore far/small detections, decrease if robot ignores nearby fuel |
| `FUEL_INTAKE_AREA_THRESHOLD` | Tune until robot stops at the right distance for intake |
| `FUEL_ROTATION_GAIN` | Increase if rotation toward target is sluggish, decrease if it oscillates |
| `FUEL_APPROACH_SPEED_FRACTION` | Reduce if robot approaches too fast and overshoots |

Watch **SmartDashboard** for `Fuel/TX Error`, `Fuel/Area`, and `Fuel/Has Target` while testing.

---

## How Accuracy Works

The model gives each detection a **confidence score**. Limelight only shows detections above a confidence threshold (configurable in the dashboard). If the model is detecting the wrong things:

- **False positives** (seeing fuel when there isn't any): add training images of those backgrounds with no fuel labeled
- **Missing detections** (not seeing fuel when it's there): add training images of those scenarios with fuel labeled

Retraining with more targeted images fixes most accuracy problems. It usually takes 2-3 rounds of collect, label, train, and test before a model is competition-ready.

---

## Common Problems

**Model not loading on Limelight**: wrong file format. Check docs.limelightvision.io for the current Hailo export format.

**No detections on real field**: training images were taken in different lighting. Recollect images under similar lighting and retrain.

**Robot rotates toward wrong object**: model is detecting something yellow that isn't fuel. Add negative examples (images of yellow field elements with no bounding boxes).

**Detections flicker rapidly**: confidence threshold too low in Limelight dashboard, or object is right at the edge of detection range. Raise threshold or raise `FUEL_MIN_AREA`.

---

Team 8041 - Lake City Ultrabots, 2026
