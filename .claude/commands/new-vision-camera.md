# /new-vision-camera — Add a camera to the Vision subsystem

Use this playbook to wire a PhotonVision or Limelight camera into the `Vision` subsystem.
Both backends share the same `VisionIO` / `VisionFactory` pattern.

---

## Step 0 — Prerequisites

- Vision subsystem already registered in `RobotContainer` via `VisionFactory.build(...)`.
  If not, go to **Step 1** first.
- Camera is physically mounted and its NT table name / PhotonVision pipeline name is known.
- `robotToCamera` transform is measured (robot origin → camera lens, in robot coordinates).

---

## Step 1 — Register Vision in RobotProfile (first camera only)

Vision is robot-specific config, so it belongs in `RobotProfile`, not `RobotContainer`.
Override `createVision(AkitSwerveDrive drive)` in `src/main/java/org/frc5010/examples/ExampleRobotProfile.java`:

```java
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import org.frc5010.common.vision.CameraConfig;
import org.frc5010.common.vision.Vision;
import org.frc5010.common.vision.VisionFactory;

@Override
public Vision createVision(AkitSwerveDrive drive) {
    AprilTagFieldLayout layout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

    Vision vision = VisionFactory.build(
        drive::addVisionMeasurement,
        // IMPORTANT: use the TRUE physics position, not the estimator.
        // If you pass drive::getPose and then inject an estimator error (e.g. push test),
        // the camera sim will follow the wrong position and stop detecting tags.
        () -> drive.getSimulatedPose().orElse(drive.getPose()),
        drive::getRotation,        // heading supplier (for Limelight MT2)
        new CameraConfig[] { /* cameras — see Step 2 */ });

    // Publish static tag poses to Field2d for Glass AprilTag image overlays.
    layout.getTags().forEach(tag ->
        drive.getField2d()
             .getObject("Field Tag " + tag.ID)
             .setPose(tag.pose.toPose2d()));

    return vision;
}
```

`SwerveRobotContainer` calls `profile.createVision(drive)` automatically after `createDrive()`
and stores the result in `protected Vision vision`. `SwerveVisualTest` reads it for Step 6.

---

## Step 2 — Configure the camera

### PhotonVision camera

```java
new CameraConfig.Builder("photon_front")       // must match PhotonVision dashboard name
    .robotToCamera(new Transform3d(
        new Translation3d(0.30, 0.0, 0.20),    // x forward, y left, z up (metres)
        new Rotation3d(0, Math.toRadians(-15), 0)))
    .backend(CameraConfig.Backend.PHOTON)
    .stdDevFactor(1.0)   // 1.0 = default trust; >1 = less trust, <1 = more trust
    .build()
```

### Limelight camera

```java
new CameraConfig.Builder("limelight")          // must match Limelight NT table name
    .robotToCamera(new Transform3d(
        new Translation3d(-0.30, 0.0, 0.50),
        new Rotation3d(0, Math.toRadians(20), Math.PI)))
    .backend(CameraConfig.Backend.LIMELIGHT)
    .stdDevFactor(0.8)   // tune to match relative confidence vs other cameras
    .build()
```

**Notes on `robotToCamera`:**
- Measured from the robot's geometric centre to the camera lens.
- Positive X = robot forward, positive Y = robot left, positive Z = up.
- The rotation is the camera's orientation relative to the robot (not the inverse).
- For Limelights the coprocessor does its own calibration but this field is still
  required for documentation and potential single-tag fallback.

---

## Step 3 — Verify SIM mode

`VisionFactory` wires `VisionIOSim` for PHOTON cameras in SIM mode automatically.
Limelight cameras get a silent no-op IO in SIM (no PhotonVision sim equivalent).

To confirm a PhotonVision camera works in sim:

1. Run `.\gradlew.bat simulateJava`
2. Open AdvantageScope → `Vision/Camera0/RobotPoses` should show a moving 3D pose
   when the robot is near an AprilTag.
3. `Vision/Camera0/Accepted` should have a non-empty array when a tag is visible
   and not rejected.

---

## Step 4 — Add a Layer 2 test (optional but recommended)

File: `src/test/java/org/frc5010/common/subsystem/VisionSubsystemTest.java`
(add a new `@Test` method to the existing class)

Stub pattern — inject a known observation:

```java
@Test
void myNewCameraAcceptsMultiTagObservation() {
    CameraConfig myCfg = new CameraConfig.Builder("my_cam")
        .robotToCamera(MY_TRANSFORM)
        .stdDevFactor(0.8)
        .build();

    List<Pose2d> received = new ArrayList<>();
    Vision vision = new Vision(
        (pose, ts, s) -> received.add(pose),
        LAYOUT,
        new CameraConfig[]{myCfg},
        new VisionIO() {
            @Override public void updateInputs(VisionIOInputs inputs) {
                inputs.connected               = true;
                inputs.observationTimestamps   = new double[] {1.0};
                inputs.observationPoses        = new Pose3d[]  {VALID_POSE};
                inputs.observationAmbiguities  = new double[] {0.05};
                inputs.observationTagCounts    = new int[]    {3};
                inputs.observationTagDistances = new double[] {2.0};
                inputs.observationTypes        = new int[]    {PoseObservationType.PHOTONVISION.ordinal()};
                inputs.tagIds                  = new int[]    {7, 8, 9};
            }
        });

    vision.periodic();
    stepOneCycle();

    assertEquals(1, received.size(), "Multi-tag observation should be accepted");
}
```

**Key gotcha:** `VisionIO` is NOT a `@FunctionalInterface` (its `updateInputs` method has
a `default` implementation, giving it zero abstract methods). Always use anonymous inner
classes — not lambdas — when stubbing `VisionIO` in tests.

---

## Step 4b — Add a Layer 3 headless test (full camera sim, CI-safe)

`VisionIOSim` uses `PhotonCameraSim` which is pure math — no display window needed. Use this
pattern in a new `@Test` inside `VisionSimIntegrationTest` (extend `SimTestBase`, follow the
`AkitSwerveDriveSimPhysicsTest` teardown pattern to reset `SimulatedArena`).

**Position rule:** pick a robot pose where the front camera (facing +X) has a tag with
`yaw = 180°` (face pointing toward −X) in front of it. For 2026 Rebuilt Welded:
- Tags 25 and 26 are at (4.02 m, 4.04–4.39 m, yaw=180°) — robot at (2.0, 4.0, 0°) sees both.
- Distance ≈ 1.7 m, safely within the 90° FOV of `PERFECT_90DEG`.

```java
// Per-cycle order in Layer 3 vision tests:
drive.simulationPeriodic();  // advance physics
drive.periodic();            // read physics → estimator
vision.periodic();           // camera sim → addVisionMeasurement
stepOneCycle();              // advance FPGA clock 20 ms
```

See `VisionSimIntegrationTest` for the full working example.

---

## Step 5 — Tune std-dev factors

After deploying to the real robot, watch `Vision/Camera*/Accepted` in AdvantageScope.
Adjust `stdDevFactor` in `CameraConfig`:
- Start at `1.0` for all cameras.
- If a camera is noisy (many rejected, or drive estimates jitter), increase its factor (e.g. `1.5`).
- If a camera is highly accurate and you want the estimator to trust it more, decrease (e.g. `0.5`).

The std-dev model already scales with `distance² / tagCount`, so closer / multi-tag
detections are trusted more automatically. `stdDevFactor` is only a per-camera bias on top.

---

## Reference — rejection criteria (Vision.java)

An observation is rejected (not sent to the estimator) if:
- `tagCount == 0`
- `tagCount == 1 && ambiguity > 0.3`
- `|pose.Z| > 0.75 m` (robot flying)
- `pose.X < 0` or `> fieldLength` or `pose.Y < 0` or `> fieldWidth`

Rejected observations are still logged to `Vision/Camera*/Rejected` for debugging.

---

## Reference — key files

| File | Purpose |
|------|---------|
| `VisionIO.java` | Interface + `@AutoLog VisionIOInputs` (parallel arrays) |
| `CameraConfig.java` | Immutable camera config, Builder |
| `VisionFactory.java` | REAL/SIM/REPLAY wiring |
| `VisionIOPhoton.java` | PhotonVision multi-tag + single-tag |
| `VisionIOLimelight.java` | YALL MegaTag 1 (NT queue) + MegaTag 2 |
| `VisionIOSim.java` | PhotonCameraSim (extends VisionIOPhoton) |
| `Vision.java` | Subsystem: filter, std-dev model, consumer call |
| `VisionSubsystemTest.java` | Layer 2 tests for Vision filtering logic |
