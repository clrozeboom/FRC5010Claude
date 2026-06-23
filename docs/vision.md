# Vision

The vision subsystem follows the same AdvantageKit IO pattern as the drive subsystem: a single factory wires backend-specific IO implementations behind a common interface, and the subsystem filters and forwards observations to the drive's pose estimator.

---

## Overview

```
CameraConfig[] (one per camera, Builder pattern)
        │
        ▼
VisionFactory.build(consumer, poseSupplier, headingSupplier, configs)
        │
        ├─ REAL  → VisionIOPhoton  (PhotonVision, multi-tag PnP)
        │          VisionIOLimelight (YALL, MegaTag 1 + MegaTag 2)
        ├─ SIM   → VisionIOSim    (extends VisionIOPhoton, PhotonCameraSim)
        │          (Limelight → no-op; no PhotonVision sim equivalent)
        └─ REPLAY→ no-op VisionIO (AKit replays logged inputs automatically)
                │
                ▼
         Vision (SubsystemBase)
          ├─ filters bad observations (ambiguity, Z error, field boundaries)
          ├─ scales std devs: distance²/tagCount × stdDevFactor
          │    MegaTag 2: ½ linear, 1e6× angular (heading locked)
          └─ calls consumer (drive::addVisionMeasurement) for accepted poses
```

---

## Key design decisions

- **`@AutoLog` parallel arrays** — `VisionIOInputs` uses parallel primitive/struct arrays (`double[]`, `Pose3d[]`, `int[]`) instead of a `PoseObservation[]` record. AdvantageKit's annotation processor only serializes WPILib struct types; custom records cause the field to be typed `Object[]` and break all accessors in `Vision.periodic()`.
- **`VisionIO.updateInputs` is `default`** — allows `new VisionIO() {}` no-op for REPLAY/Limelight-in-SIM without subclassing. Consequence: `VisionIO` is NOT a `@FunctionalInterface`; use anonymous inner classes (not lambdas) in tests.
- **The profile chooses the field; everything follows** — `RobotProfile.getAprilTagFieldLayout()` is the authority. `SwerveRobotContainer` pushes it into the shared `AprilTags` holder before building the drive/vision, and `VisionFactory` reads `AprilTags.aprilTagFieldLayout` rather than loading its own `kDefaultField`. So pose estimation and any field-geometry helpers share the one layout the profile picked and can never diverge. The default is `kDefaultField` (welded); a profile can override to another variant (e.g. `k2026RebuiltAndymark` for the AndyMark field).
- **`poseSupplier` must be the TRUE physics position** — `VisionIOSim` uses this supplier to place the simulated camera. Always pass `() -> drive.getSimulatedPose().orElse(drive.getPose())`, NOT `drive::getPose`. If you use the estimator pose and then inject an estimator error (e.g. push-correction test), the camera sim will be moved to the wrong position and stop detecting tags — breaking the very correction you're testing.
- **One `VisionSystemSim` for all cameras** — PhotonVision's `VisionSystemSim` already models any number of cameras (`addCamera` per camera; one `update(pose)` advances all). `VisionFactory` therefore builds a single shared `VisionSystemSim` (wrapped by `SharedVisionSim`) and registers every PHOTON sim camera into it, instead of one simulator per camera. The sim is stepped at most once per loop (de-duplicated by FPGA timestamp inside `SharedVisionSim.updateOncePerLoop`, so it is order-independent of which camera's `updateInputs` runs first), and a single "Sim Field" debug widget shows all cameras. A `VisionIOSim` constructed standalone (single camera, e.g. in a unit test) still gets its own dedicated simulator.
- **MegaTag 1 via NT queue** — `megatag1Subscriber.readQueue()` drains every frame since the last cycle so no poses are dropped between 20 ms loops.
- **Orientation via YALL `withRobotOrientation`** — `limelight.getSettings().withRobotOrientation(new Orientation3d(rot3d, zero))` sets the NT key `robot_orientation_set` and flushes; the Limelight uses this to lock its heading for MegaTag 2.

---

## Usage example

```java
Vision vision = VisionFactory.build(
    drive::addVisionMeasurement,
    () -> drive.getSimulatedPose().orElse(drive.getPose()),  // TRUE physics position, not estimator
    drive::getRotation,
    new CameraConfig[] {
        new CameraConfig.Builder("photon_front")
            .robotToCamera(FRONT_CAM_TRANSFORM)
            .backend(CameraConfig.Backend.PHOTON)
            .build(),
        new CameraConfig.Builder("limelight")
            .robotToCamera(REAR_CAM_TRANSFORM)
            .backend(CameraConfig.Backend.LIMELIGHT)
            .stdDevFactor(0.8)   // trust this camera more
            .build()
    });
```

See `/new-vision-camera` for the step-by-step wiring guide.

---

## Shared field layout (`AprilTags`)

`org.frc5010.common.vision.AprilTags` is a tiny holder for the season's AprilTag field layout
so **field-geometry helpers can resolve tag poses by ID without each loading their own copy**:

```java
import org.frc5010.common.vision.AprilTags;

Optional<Pose3d> hubFace = AprilTags.aprilTagFieldLayout.getTagPose(26);
double fieldLength       = AprilTags.aprilTagFieldLayout.getFieldLength();
```

- `public static AprilTagFieldLayout aprilTagFieldLayout` — the **active** layout. Defaults to
  `AprilTagFields.kDefaultField` (welded) as a fallback, but is normally **set from the robot
  profile** — see below.
- `setAprilTagFieldLayout(layout)` — replace the active layout. `SwerveRobotContainer` calls
  this with `profile.getAprilTagFieldLayout()`; you can also call it directly for a practice/lab
  field. Call before anything reads field constants or builds vision.

**The `RobotProfile` is the authority on the field.** `SwerveRobotContainer` pushes
`profile.getAprilTagFieldLayout()` into this holder before it builds the drive and vision, so
`VisionFactory` (pose estimation) and any field-geometry helpers that read this holder all
resolve against the one layout the profile picked — they never diverge. A profile that competes
on a non-default field overrides `getAprilTagFieldLayout()` to return that variant.

> Field constants are often computed in static initialisers when their class first loads, so the
> profile must set the field **before** any such access — which the container does (it sets the
> layout in its constructor, ahead of the subsystems that read field geometry).

---

## Web UI tag highlight

When the web UI is active (`-PwebUI`), the field view highlights every AprilTag currently
in view **red** (default tags are teal). The feed is:

`Vision.periodic()` collects the union of `VisionIOInputs.tagIds` across all cameras into a
`volatile int[]` → `Vision.getVisibleTagIds()` → bound by `SwerveRobotContainer` via
`WebControl.bindVision(vision)` → surfaced in `/api/state` as the `visibleTags` array →
`drawAprilTags()` in `index.html` draws the matching tags red (tick, border, glow, label).

The binding only happens when the container has a non-null `vision`; with no vision subsystem
`visibleTags` is always `[]` and all tags render teal.
