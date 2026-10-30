# Mission Capability Development Roadmap

## Context

We are building a mission capability for autonomous drones for surveillance, fire, and rescue. We have the standard perception pipeline of object detection, object identification, and tracking, but the OSS YOLO is producing very poor detection scores with lots of misidentified objects. 

We need to plan out a mission capability progression where we can start small and troubleshoot the perception quality. The simplest mission capability we can think of is identify one, specific object, and track it, that is, follow this object at a safe distance and match its direction and speed. 

The next mission capability would be to work in a world where there are two objects, first without interference, second with interference. This is three missions that build on top of each other and test perception at a progressively more difficult level, but very incremental. 

We need to generate a sequence of such progressively more difficult mission capabilities from these three that build on the preception and control of the previous missions but increase the difficulty. We want to end up with a drone capability that could identify an object, such as a person, bicyclist, or a car, and track it through highly complex environments, such as an airport, a city, or an urban environment. 

## Assessment of early experiments

It is common to experience low fidelity object detection with out-of-the-box YOLO models for drone applications. Standard open-source object detection models are typically trained on datasets with ground-level camera angles (like COCO), so when you put them in the air, the top-down or angled perspectives immediately cause confidence scores to plummet and misidentifications to spike.

Starting small and isolating the variables between detection (finding the box) and tracking (associating the box over time) is a sensible approach. To get from a basic isolated track to a complex urban environment, we need to incrementally stress-test our perception, our data association (tracking algorithm), and our flight control.

---

### Phase 1: Foundational Perception and Control

**Mission 1: The Solitary Sentinel**

* **Description:** A single, visually distinct target (e.g., a person in a bright jacket or a brightly colored car) moves in a flat, high-contrast environment like an open field or empty parking lot. The drone must lock onto the target, maintain a fixed safe distance, and match its heading and speed.
* **Progression:** This is our baseline. It isolates basic control loops (PID or Model Predictive Control) from complex perception. It validates that the YOLO model can consistently detect the object from an aerial angle and that your camera gimbal and flight controller can keep the bounding box centered.

**Mission 2: Parallel Paths**

* **Description:** Introduce a second, visually distinct object. Both objects move independently in the field of view without their paths ever crossing. The drone is instructed to track Object A and ignore Object B.
* **Progression:** This introduces basic ID assignment. It tests whether your flight control system will get confused, average the positions of the two bounding boxes, or randomly snap between the two targets.

**Mission 3: The Crossroads**

* **Description:** The two objects from Mission 2 now cross paths, causing the target being tracked to be briefly occluded by or intersect with the distractor object.
* **Progression:** This tests your basic tracking and data association layer (e.g., Kalman filters, SORT). Can the system maintain the correct ID across a brief visual intersection without swapping IDs when the bounding boxes overlap?

---

### Phase 2: Perception Stress and Feature Extraction

**Mission 4: The Chameleon (Similar Classes & Variable Backgrounds)**

* **Description:** The target is now surrounded by 3-5 distractor objects of the *exact same class* (e.g., a group of pedestrians). The environment also includes variable lighting, such as the target moving from bright sunlight into deep building shadows.
* **Progression:** This stresses the YOLO classifier and your tracking algorithm's Re-Identification (ReID) capabilities. You are moving from tracking "a moving thing" to feature-matching "this specific thing" (using algorithms like DeepSORT or ByteTrack). It also tests if your camera's auto-exposure and your model's robustness can handle sudden lighting changes without dropping the bounding box.

**Mission 5: Hide and Seek (Prolonged Occlusion)**

* **Description:** The target walks behind a large visual obstruction (a dense tree canopy or a small building), completely disappearing from the camera's view for 5 to 10 seconds before emerging on the other side.
* **Progression:** This tests memory and predictive control. The drone can no longer rely on visual frame-to-frame matching. It must predict the target's trajectory using its last known velocity and heading, hover or continue moving along that predicted path, and successfully re-acquire the target's ID based on visual embeddings when it reappears.

---

### Phase 3: Complex Environments and Autonomous Navigation

**Mission 6: The Obstacle Course**

* **Description:** The target moves through a moderately dense environment with static vertical obstacles (e.g., light poles, trees, parked cars). The drone must maintain its track on the target while actively navigating around these obstacles to maintain line of sight and avoid collisions.
* **Progression:** This is a major leap. It fuses the 2D perception pipeline with active 3D path planning and obstacle avoidance (e.g., using depth cameras or LiDAR). The flight controller must now juggle target vectoring with its own self-preservation, recalculating flight paths on the fly.

**Mission 7: The Dynamic Gauntlet**

* **Description:** Introduce dynamic distractors. The target navigates a space with other moving elements that are *not* the target class (e.g., tracking a bicyclist while random cars, loose dogs, and pedestrians move through the frame at different speeds).
* **Progression:** This tests the system's ability to filter out high volumes of "noise." It ensures that your YOLO model isn't generating false positives on objects moving rapidly through the periphery of the frame, which could hijack the tracking system.

**Mission 8: The Apex Tracker (Urban / Airport Simulation)**

* **Description:** The ultimate test. The target navigates a highly complex, dense environment (a mock city block or a busy transport hub exterior). This includes unpredictable target behavior (stopping, sprinting, turning sharply), dense crowds, varying elevations (stairs/ramps), and heavy visual clutter.
* **Progression:** This brings everything together. It requires high-confidence ReID, dynamic flight path recalculation, rapid edge-compute processing speed to handle dozens of bounding boxes per frame, and seamless integration between the perception, tracking, and flight control stacks.

---

> **A quick note on our current YOLO setup:** To move past Phase 1, we will to pair YOLO with a robust tracker that uses deep appearance features (like DeepSORT or StrongSORT) so the drone remembers what the target *looks* like, not just where it was a fraction of a second ago.

Would you like to explore strategies for fine-tuning your YOLO model on aerial datasets to fix the immediate perception issues, or would you rather dive into the architecture of the tracking algorithms you'll need for Mission 3 and beyond?

## Improving Object Detection

Most teams get stuck in a rut trying to tweak YOLO's architecture or hyperparameters to fix aerial perception issues. But, the root cause is almost always the data domain gap: standard YOLO is trained on ground-level, horizontal images, while a drone sees top-down, angled, and highly varied perspectives.

3LC shifts the paradigm from model-centric to **data-centric AI**. Instead of flying blind, their methodology allows you to "eavesdrop" on your object detection model while it trains, letting the model tell you exactly which images are causing quality issues.

We should leverage 3LC’s quality assessment methodology to fine-tune our YOLO model specifically for our mission progression:

### The 3LC Fine-Tuning Loop for Aerial Data

**Step 1: The Baseline "Listening" Run**
Instead of just training YOLO and looking at an aggregate mAP (mean Average Precision) score at the end, we wrap our training script with 3LC's integration (e.g., using the `TLCYOLO` wrapper if using Ultralytics).

* **The Action:** Train the baseline YOLO model on a select set of aerial datasets with mission specific content.
* **The 3LC Magic:** During training, 3LC continuously collects granular, per-sample metrics—tracking confidence scores, IoU (Intersection over Union), embeddings, and false positives/negatives for *every single bounding box* across every epoch.

**Step 2: Diagnosis via the 3LC Dashboard**
Once the baseline run is complete, open the 3LC dashboard. This is where we diagnose exactly why the drone is misidentifying objects in Phase 1 and 2 of our missions.

* **Finding Missing Labels (The "Phantom Object" Problem):** Filter for **High-Confidence False Positives**. In aerial imagery, a human annotator might easily miss a person standing in a shadow. If YOLO detects a person there with 95% confidence, but since there is no ground-truth bounding box, 3LC flags it as a false positive. We should be able to see these and realize the model is right and your human annotator was wrong.
* **Fixing Sloppy Bounding Boxes:** Filter for True Positives with **Low IoU**. Aerial objects tend to be small. If the bounding box is slightly off, the model learns bad spatial representations. We must quickly identify and tighten these boxes.
* **Tracking Embedding Movements:** Look at the samples that travel the furthest in embedding space between epochs. These are our "hard" samples—the weird camera angles, the glares, the occlusions.

**Step 3: Immediate Data Enhancement**
Traditional pipelines requires data exports, annotation team assessment, wait weeks, and re-import. 3LC allows for interactive data editing.

* **The Action:** With a few clicks, we can convert those high-confidence False Positives into actual ground-truth labels. We can also re-weight our dataset. If the drone struggles with Mission 4 (Variable Backgrounds), we can find the images with deep building shadows and assign them a higher training weight (e.g., `weight = 2.0`), forcing the model to prioritize learning those edge cases.

**Step 4: Iterative Retraining**
We don't move your data; 3LC creates a lightweight, sparse revision. You simply restart the training run.

* **The Result:** Because we removed the noise (bad labels) and amplified the signal (hard aerial edge cases), the next YOLO model will converge faster, require less overall data, and perform with significantly higher accuracy when deployed to the drone's edge compute.

---

### Tying it Back to the Mission Progression

As we move through the mission capabilities discussed earlier, a 3LC methodology will evolve:

* **For Missions 1-3 (Foundational Tracking):** Use 3LC to ensure datasets have hyper-accurate bounding boxes for specific target classes from various drone altitudes and pitch angles.
* **For Missions 4-5 (Occlusions & Shadows):** Heavily utilize 3LC's embedding analysis to find the "hard" samples where targets are partially obscured by trees or vehicles, and up-weight them in the training loop.
* **For Missions 6-8 (Urban/Complex Environments):** Introduce highly cluttered aerial imagery into 3LC. Filter out the "easy" background samples and ruthlessly focus the model's training compute on distinguishing the target from dynamic distractors.

By using the model's own struggles to guide dataset curation, we'll build a highly specialized, highly efficient perception stack.

## Sourcing the raw aerial video data

Sourcing the right initial data is the make-or-break step for the entire pipeline. If we feed the YOLO model ground-level images, we will spend months fighting the "domain gap." Because we are building a tracking system (not just static detection), we also specifically need sequential video data, not just random, disconnected images.

Here is a three-pronged strategy for sourcing that initial raw aerial video data to fuel your 3LC data-centric loop and get through your mission progression:

### Strategy 1: High-Yield Open-Source Datasets (The "Cold Start")

We don't need to fly a single drone to get a baseline YOLO model off the ground. Several massive academic and commercial datasets exist specifically for drone-based object detection and tracking.

* **VisDrone (Vision Meets Drone):** This is the gold standard for starting out. It contains hundreds of video clips captured by various drone platforms across dozens of cities, targeting pedestrians, cars, and bicycles.
* **UAVDT (Unmanned Aerial Vehicle Benchmark: Object Detection and Tracking):** Excellent for vehicle tracking, varying weather conditions, and different flying altitudes.
* **Okutama-Action:** A great dataset specifically for human detection and action recognition from an aerial perspective.
* **Stanford Drone Dataset:** Top-down footage of university campuses, heavily featuring pedestrians, bicyclists, and skateboarders navigating around each other (perfect for **Phase 2: Missions 4 & 5**).

**How to leverage 3LC here:** These datasets are massive and often contain labeling errors or scenarios we don't care about (e.g., footage from 400 feet up when the drone will fly at 50 feet). We can import a subset of VisDrone into 3LC, train the baseline, and use the 3LC dashboard to ruthlessly prune the dataset down to only the altitudes, angles, and target classes relevant to your specific missions.

### Strategy 2: Synthetic Data Generation (The "Edge-Case Factory")

Open-source data is great for baselines, but it rarely contains the exact complex scenarios we need for **Phase 3 (Missions 6-8)**, like extreme lighting changes, specific occlusion events, or a dense airport environment.

By using game engines, you can generate photorealistic drone footage with *mathematically perfect* bounding box labels.

* **NVIDIA Isaac Sim / Omniverse:** Highly tailored for robotics. We can simulate your exact camera sensor (FOV, lens distortion, noise) and fly a virtual drone through complex, physically accurate 3D environments.
* **Unreal Engine 5:** Provides options to purchase high-fidelity 3D assets (like a city block or forest) and script a virtual camera to follow 3D human models.
* **The Benefit:** Want 10,000 perfectly labeled frames of a bicyclist moving in and out of building shadows at dusk? We can generate that in an afternoon. No human annotation required.

**How to leverage 3LC here:** When we mix synthetic data with real data, models sometimes cheat by learning the "synthetic artifacts" rather than the object. 3LC's embedding visualization allows us to see if the model is grouping synthetic images separately from real images. If they are clustered together in the embedding space, the synthetic data is working perfectly.

### Strategy 3: Targeted Flight Operations (The "Ground Truth")

Eventually, we must train on the exact hardware and environments where you will operate. Since we have a structured mission progression, our flight ops can map perfectly to it.

* **Fly the Missions Manually:** Have a pilot manually fly Mission 1 (Solitary Sentinel) and Mission 2 (Parallel Paths) using the exact drone, camera payload, and target classes.
* **Vary the Variables Incrementally:** Record 4K video at 30fps. Fly at 20 feet, 50 feet, and 100 feet. Fly with the gimbal pitched at 45 degrees, 60 degrees, and straight down (90 degrees).
* **Frame Extraction:** We don't need to annotate 30 frames per second. Extract frames at 2 to 5 FPS. This provides enough sequential overlap for the tracker to learn frame-to-frame association without drowning in redundant labeling costs.

**How to leverage 3LC here:** Use the model trained on Open-Source + Synthetic data to auto-label the new custom flight footage. Import those auto-labels into 3LC. The dashboard will immediately flag the instances where the model's confidence drops. Only spend human annotation budget fixing those specific "hard" frames, rather than paying someone to label empty grass.

---

**The Recommended Path Forward:**

1. Download a subset of **VisDrone** (e.g., 5,000 sequential frames of pedestrians and cars).
2. Format the labels into YOLO format.
3. Run baseline training with the 3LC integration to clean it up.

