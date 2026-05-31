# Student Setup Guide — Fork, Configure, and Deploy Your Robot

This guide walks you through taking this swerve library and wiring it to your own robot — from forking the repository to your first real-hardware drive test. No prior AdvantageKit experience is required.

**Estimated time:** 2–3 hours for the simulation step; another 1–2 hours to wire and deploy to hardware.

---

## What you will end up with

- A working swerve drive subsystem, verified in simulation, tuned with your robot's real measurements
- Keyboard-controlled simulation you can run without any physical robot
- A codebase ready to deploy and drive at your next practice

---

## Prerequisites

Install these tools before you start. All are free.

| Tool | Where to get it | Why you need it |
|------|----------------|-----------------|
| **WPILib 2026** | [docs.wpilib.org → Installation](https://docs.wpilib.org/en/stable/docs/zero-to-robot/step-2/wpilib-setup.html) | Robot framework + VS Code + Java 17 bundled together |
| **Git** | [git-scm.com](https://git-scm.com/downloads) | Cloning and pushing code |
| **Phoenix Tuner X** | Windows Store or [CTR Electronics](https://v6.docs.ctr-electronics.com/en/latest/docs/tuner/index.html) | Required only if your robot uses TalonFX/Kraken motors |
| **AdvantageScope** | [github.com/Mechanical-Advantage/AdvantageScope](https://github.com/Mechanical-Advantage/AdvantageScope/releases) | 3D visualization of your robot in simulation |

> **WPILib bundles VS Code and Java 17.** Use the VS Code that WPILib installs (it has the WPILib extension pre-loaded), not a separate VS Code installation. The WPILib installer puts it at `C:\Users\Public\wpilib\2026\vscode\Code.exe` on Windows.

---

## Step 1 — Fork the repository

1. Go to [github.com/clrozeboom/FRC5010Claude](https://github.com/clrozeboom/FRC5010Claude) (or your team's fork of this library).
2. Click **Fork** (top-right corner) → **Create fork**.
3. Your fork lives at `github.com/<your-team-org>/FRC5010Claude`. This is your robot's codebase.

---

## Step 2 — Clone to your computer

Open a terminal (PowerShell on Windows, Terminal on Mac/Linux) and run:

```powershell
git clone https://github.com/<your-team-org>/FRC5010Claude.git
cd FRC5010Claude
```

Then open the folder in WPILib's VS Code:

```powershell
code .
```

VS Code will show a Gradle import notification — click **Yes** to let it download dependencies. This can take 5–10 minutes on the first open.

> **If the import hangs or fails:** make sure your computer can reach `frcmaven.wpi.edu` and `services.gradle.org`. School network firewalls sometimes block these. Try from a personal hotspot if needed.

---

## Step 3 — Collect your robot's measurements

Before touching any code, gather these numbers from your physical robot. Write them down — you will type them into one file.

### Physical geometry

Measure from the **centre of one wheel** to the **centre of the opposite wheel**. Do not measure edge-to-edge.

| What to measure | Field name | Typical range |
|-----------------|-----------|---------------|
| Left-to-right distance between front and rear wheel centres | `trackWidth` | 18–24 in |
| Front-to-back distance between left and right wheel centres | `wheelBase` | 18–24 in |
| Actual wheel radius (measure a worn wheel, not the spec) | `wheelRadius` | 1.75–2.25 in |

> **Tip:** If your frame is square and symmetric, `trackWidth` equals `wheelBase`.

### Weight

Weigh your robot with bumpers and battery installed. Use a postal scale or a luggage scale hung from the frame.

| What to measure | Field name | Typical range |
|-----------------|-----------|---------------|
| Total weight with bumpers and battery | `robotMass` | 100–130 lb |

### Bumper size (outside-to-outside)

Measure from the outside face of one bumper to the outside face of the opposite bumper.

| What to measure | Field name |
|-----------------|-----------|
| Outside-to-outside, front-to-back | `bumperLength` |
| Outside-to-outside, left-to-right | `bumperWidth` |

### CAN IDs

Look these up in Phoenix Tuner X (for TalonFX/CANcoder) or REV Hardware Client (for SparkMAX). You need:

- Drive motor CAN ID for each module (4 total)
- Steer motor CAN ID for each module (4 total)
- CANcoder (encoder) CAN ID for each module (4 total, if applicable)
- Pigeon 2 or NavX CAN ID

Module naming convention: **FL = front-left, FR = front-right, BL = back-left, BR = back-right**. "Front" is the end that faces your opponent's alliance wall during teleop.

### Starting pose

Decide where your robot will start each match on the Blue alliance side. Look at the field layout for this year's game. You need:

- X: distance in metres from the Blue alliance wall (left wall when facing the field from Blue side)
- Y: distance in metres from the bottom wall

For 2026 REEFSCAPE, a typical starting pose is around x=1.5 m, y=4.0 m for a centre start. Refer to your team's game strategy.

---

## Step 4 — Edit `RealRobotProfile.java`

This is the one file that describes your robot. Open it in VS Code:

```
src/main/java/frc/robot/RealRobotProfile.java
```

### 4a — Fill in `SwerveConstants`

Find the block that starts with `private static final SwerveConstants CONSTANTS`. Replace each placeholder value with your robot's real numbers.

```java
private static final SwerveConstants CONSTANTS = new SwerveConstants.Builder()
    // --- Hardware type ---
    .moduleType(ModuleType.TALON_FX)   // ← TALON_FX for Falcon/Kraken; SPARK_TALON for NEO drive
    .gyroType(GyroType.PIGEON2)        // ← PIGEON2 or NAVX
    .gyroCanId(1)                      // ← CAN ID of your Pigeon 2 (ignored for NavX)

    // --- Physical geometry (use your measurements from Step 3) ---
    .trackWidth(Inches.of(22.75))      // ← left-to-right wheel centre distance
    .wheelBase(Inches.of(22.75))       // ← front-to-back wheel centre distance
    .wheelRadius(Inches.of(2.0))       // ← actual worn radius of your wheels

    // --- Weight and bumpers (use your measurements from Step 3) ---
    .robotMass(Pounds.of(125))         // ← total weight with bumpers and battery
    .bumperLength(Inches.of(30))       // ← outside-to-outside front-to-back
    .bumperWidth(Inches.of(30))        // ← outside-to-outside left-to-right

    // --- CAN IDs: [driveId, steerId, encoderId] per module ---
    .frontLeftIds(1, 2, 3)             // ← FL drive, steer, encoder
    .frontRightIds(4, 5, 6)            // ← FR drive, steer, encoder
    .backLeftIds(7, 8, 9)              // ← BL drive, steer, encoder
    .backRightIds(10, 11, 12)          // ← BR drive, steer, encoder

    // --- CAN bus ---
    .canBusName("")                    // ← "" for RIO bus; your CANivore name for Phoenix Pro
    .odometryFrequency(Hertz.of(100))  // ← 100 Hz for RIO bus; 250 Hz for CANivore
    .build();
```

> **Common mistake:** `wheelRadius` is the *worn-down* radius, not the nominal size printed on the wheel. A "4-inch" wheel measures closer to 1.9–1.95 in radius after use. Rolling the robot one metre and measuring actual travel is the most accurate method.

### 4b — Set the starting pose

Find the line:
```java
private static final Pose2d BLUE_START = new Pose2d(1.5, 2.0, new Rotation2d());
```

Replace `1.5` and `2.0` with your robot's Blue-alliance X and Y starting coordinates (in metres). `new Rotation2d()` means the robot faces forward (0°); adjust the angle if your robot starts rotated.

### 4c — Choose the right `ModuleType`

| Your drive motors | Your steer motors | Use |
|------------------|------------------|-----|
| TalonFX (Falcon 500 or Kraken X60) | TalonFX | `ModuleType.TALON_FX` |
| SparkMAX (NEO or NEO 550) | TalonFX | `ModuleType.SPARK_TALON` |
| SparkMAX | SparkMAX | `ModuleType.SPARK_MAX` |

### 4d — Save and verify it compiles

Press **Ctrl+Shift+B** in VS Code to build, or run in terminal:

```powershell
# Windows
.\gradlew.bat build -x test

# macOS / Linux
./gradlew build -x test
```

Fix any red underlines in VS Code before moving on. The most common error is a mismatched import — VS Code's **Quick Fix** (Ctrl+.) will offer to add it automatically.

---

## Step 5 — Run the simulation

Simulation lets you verify your constants make sense before touching the robot.

```powershell
# Windows
.\gradlew.bat simulateJava

# macOS / Linux
./gradlew simulateJava
```

### Running locally (VS Code on your computer)

WPILib's **Glass** window opens automatically. You will see:

- A **Driver Station** panel (bottom-left or a separate floating panel) — controls enabled/disabled state and match mode
- A **Field2d** widget showing your robot's position on the field
- A **NetworkTables** panel with live signal values

**Connect AdvantageScope for a 3D view:**

1. Open AdvantageScope (install from [Mechanical Advantage's GitHub releases page](https://github.com/Mechanical-Advantage/AdvantageScope/releases) if you haven't already).
2. Click **File → Connect to Robot**, enter `localhost` as the host, leave the port at the default (`5810`).
3. In the left signal panel, find `RealOutputs/Drive/Pose` and drag it onto the 3D field canvas.

You should see a robot-shaped box on the field at your configured starting position.

**Enable the robot in Glass:**

1. In Glass, find the **Driver Station** panel (it may be a separate floating window titled "Driver Station").
2. Set the mode to **Teleop** and click **Enable**.
3. Use your keyboard or a connected gamepad to drive. The Glass **NetworkTables** panel shows joystick axes in real time under `Joysticks/0/`.

### Visual auto-test (simplest way to verify motion)

The visual auto-test runs a scripted sequence — forward, strafe, rotate — and automatically enables the robot. Use this to confirm all four modules are wired correctly without needing a gamepad.

```powershell
# Windows
.\gradlew.bat simulateJava -PvisualTest

# macOS / Linux
./gradlew simulateJava -PvisualTest
```

Once Glass opens, set the Driver Station to **Autonomous** mode and click **Enable**. The robot drives a pre-programmed path over about 10 seconds.

### Running in Codespaces or a web environment (headless)

If you're using GitHub Codespaces or the claude.ai/code web environment, there is no display for Glass. Use the visual-test flag, which auto-enables the robot so no manual Driver Station click is needed:

```bash
xvfb-run ./gradlew simulateJava -PvisualTest
```

`xvfb-run` provides a virtual display so WPILib can start without crashing — you won't see Glass, but the simulation runs. To observe what's happening:

1. In VS Code's **Ports** panel, find port **5810**. Hover over it and click the globe icon to copy the forwarded address.
2. In AdvantageScope on your local laptop: **File → Connect to Robot**, paste the forwarded hostname (everything after `https://`, without the slash), port `5810`.
3. AdvantageScope receives the live data and renders the robot in 3D.

Alternatively, the simulation writes a `.wpilog` file to `logs/` when it finishes. Download it via the VS Code file explorer and open it in AdvantageScope with **File → Open Log** to review the run offline.

**If the robot does not move during the visual test:** confirm you used `-PvisualTest`; without it, the robot starts disabled and nothing will happen until you manually click Enable.

**If the motion looks wrong (spinning in place, going sideways when commanded forward):** your FL/FR/BL/BR module assignments are probably in the wrong order. See the [Troubleshooting](#troubleshooting) section.

---

## Step 6 — Wire hardware IO for the real robot

The simulation uses the factory automatically. For real hardware, you need to tell the code which motors and encoders to use.

### 6a — If you have TalonFX motors (Falcon 500 / Kraken X60)

TalonFX modules require **CTRE Phoenix Tuner X** to generate a `TunerConstants` file. This file carries your gear ratios, encoder offsets, and PID gains — information the library cannot calculate without hardware access.

#### Generate TunerConstants with Phoenix Tuner X

1. Connect your robot (USB or WiFi to the RoboRIO).
2. Open Phoenix Tuner X → **Swerve Project Generator**.
3. Fill in the wizard: select your motors, enter gear ratios, click through the encoder zeroing steps. Tuner X measures your encoder offsets automatically.
4. Click **Generate Robot Code** and save. Tuner X exports a `TunerConstants.java` file.
5. Copy `TunerConstants.java` into your project at `src/main/java/frc/robot/TunerConstants.java`.

#### Wire the real IO

In `RealRobotProfile.java`, find the `createDrive()` method. The `if (RobotBase.isReal())` block contains commented-out code. Uncomment it:

```java
@Override
public AkitSwerveDrive createDrive() {
    if (RobotBase.isReal()) {
        GyroIO gyro = new GyroIOPigeon2(CONSTANTS);
        ModuleIO[] modules = {
            new ModuleIOTalonFXReal(CONSTANTS, TunerConstants.FrontLeft),
            new ModuleIOTalonFXReal(CONSTANTS, TunerConstants.FrontRight),
            new ModuleIOTalonFXReal(CONSTANTS, TunerConstants.BackLeft),
            new ModuleIOTalonFXReal(CONSTANTS, TunerConstants.BackRight),
        };
        return new AkitSwerveDrive(CONSTANTS, gyro, modules);
    }
    // Simulation: IronMaple uses the real robot's mass/geometry for accurate physics.
    return SwerveFactory.build(CONSTANTS, BLUE_START);
}
```

Add these imports at the top of the file if VS Code shows red underlines:

```java
import org.frc5010.common.drive.swerve.akit.GyroIOPigeon2;
import org.frc5010.common.drive.swerve.akit.ModuleIO;
import org.frc5010.common.drive.swerve.akit.ModuleIOTalonFXReal;
```

### 6b — If you have SparkMAX + TalonFX (NEO drive, TalonFX steer)

Use `ModuleIOSparkTalon` instead. The four modules are identified by index (0=FL, 1=FR, 2=BL, 3=BR):

```java
if (RobotBase.isReal()) {
    GyroIO gyro = new GyroIONavX();   // or new GyroIOPigeon2(CONSTANTS)
    ModuleIO[] modules = {
        new ModuleIOSparkTalon(CONSTANTS, 0),  // FL
        new ModuleIOSparkTalon(CONSTANTS, 1),  // FR
        new ModuleIOSparkTalon(CONSTANTS, 2),  // BL
        new ModuleIOSparkTalon(CONSTANTS, 3),  // BR
    };
    return new AkitSwerveDrive(CONSTANTS, gyro, modules);
}
```

Add the imports:

```java
import org.frc5010.common.drive.swerve.akit.GyroIONavX;
import org.frc5010.common.drive.swerve.akit.ModuleIOSparkTalon;
```

---

## Step 7 — Deploy to the robot

1. Connect to the robot via USB or WiFi.
2. In VS Code, press **Ctrl+Shift+P** → type **WPILib: Deploy Robot Code** → Enter.

   Or from the terminal:
   ```powershell
   .\gradlew.bat deploy   # Windows
   ./gradlew deploy       # macOS / Linux
   ```

3. Watch the Driver Station for faults. A clean deploy shows green indicators for all CAN devices.

### First-drive checklist

Before enabling for the first time on hardware, go through this list:

- [ ] All CAN devices show green in Phoenix Tuner X (no red/orange fault lights)
- [ ] Encoder offsets from TunerX are loaded (each module's steer position reads a sensible angle)
- [ ] Robot is on a stand with wheels off the ground
- [ ] One person on the field E-stop, one person on the Driver Station
- [ ] Test each module individually: command the robot forward at 10% speed and watch which wheels spin

**If a module steers the wrong direction:** swap the `steerId` and `encoderId` for that module in `SwerveConstants`, or adjust the encoder offset in TunerX.

**If a module drives backwards:** the drive motor is inverted. Adjust the inversion flag in TunerX or `ModuleIOTalonFXReal`.

---

## Step 8 — Add a gamepad

The default wiring uses keyboard controls (WASD + ER) mapped to joystick port 0 in simulation. For a real match you need an Xbox (or similar) controller at Driver Station port 0.

The library provides `XboxConfigurableController` — a wrapper that gives each button and axis a named method and applies a chainable transform pipeline (deadzone, response curve, scaling). Override `configureBindings()` in `RobotContainer.java` to replace the keyboard drive with it:

```java
@Override
protected void configureBindings() {
    // Do NOT call super.configureBindings() — this replaces the default command entirely.
    // (The keyboard drive uses axis 2 for rotation; Xbox has the left trigger on axis 2.
    //  Using super would give you left-trigger rotation, which is not what you want.)

    XboxConfigurableController driver = new XboxConfigurableController(0);

    // Left stick = translation, right stick X = rotation.
    // negate() corrects WPILib's convention (up = negative Y, left = negative X).
    // power(2.0) gives a gentle-start response curve.
    // unitCircle() prevents diagonal inputs from exceeding the robot's max speed.
    JoystickAxis forward  = driver.leftY().negate().deadzone(0.05).power(2.0);
    JoystickAxis strafe   = driver.leftX().negate().deadzone(0.05).power(2.0);
    JoystickAxis rotation = driver.rightX().negate().deadzone(0.10);
    DriveVector translate = DriveVector.of(forward, strafe).unitCircle();

    drive.setDefaultCommand(
        Commands.run(
            () -> {
              // Flip translation direction on Red alliance so "forward" always faces
              // the opponent's wall regardless of which side you're on.
              double flip = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red
                  ? -1.0 : 1.0;
              Translation2d xy = translate.get();
              drive.runVelocityFieldRelative(new ChassisSpeeds(
                  flip * xy.getX() * drive.getMaxLinearSpeed().in(MetersPerSecond),
                  flip * xy.getY() * drive.getMaxLinearSpeed().in(MetersPerSecond),
                  rotation.getAsDouble() * drive.getMaxAngularSpeed().in(RadiansPerSecond)));
            },
            drive
        ).withName("XboxDrive")
    );

    // A button: zero the heading (useful after gyro drift)
    driver.a().onTrue(Commands.runOnce(() -> drive.setPose(
        new Pose2d(drive.getPose().getTranslation(), new Rotation2d()))));
}
```

Add the imports:

```java
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Commands;
import org.frc5010.common.drive.swerve.DriveVector;
import org.frc5010.common.drive.swerve.JoystickAxis;
import org.frc5010.common.drive.swerve.XboxConfigurableController;
```

> **Testing in simulation without a physical controller:** The visual auto-test (`-PvisualTest`) doesn't use the controller at all, so you can verify basic motion without a gamepad. For interactive keyboard testing, the default keyboard mapping (WASD for translation, E/R for rotation) is replaced by this override — use the visual test or plug in a controller.

---

## Step 9 — Add an autonomous routine (optional)

Override `getAutonomousCommand()` in `RobotContainer.java`:

```java
@Override
public Command getAutonomousCommand() {
    // Keep the visual test available when running with -PvisualTest
    if (Boolean.getBoolean("visualTest")) {
        return SwerveVisualTest.build(drive, vision, this::getAllianceStartPose);
    }
    // Replace with your PathPlanner or Choreo auto
    return AutoBuilder.buildAuto("MyAutoName");
}
```

PathPlanner must be configured separately. See the [PathPlannerLib docs](https://pathplanner.dev/home.html) for path creation and the `AutoBuilder.configure()` call.

---

## Troubleshooting

### Robot does not move in simulation

| Symptom | Most likely cause | Fix |
|---------|------------------|-----|
| Robot stays still, visual test does nothing | Robot is disabled | Click **Enable** in Glass → Driver Station |
| Visual test runs but robot does not move | Physics cycle order wrong | Do not override `simulationPeriodic()` in Robot.java — the scheduler handles it |
| Robot moves but pose stays at origin | `simulationPeriodic()` is not being called | Confirm `AkitSwerveDrive` is registered as a subsystem (it is, via `SwerveFactory`) |

### Robot moves the wrong direction

| Symptom | Most likely cause | Fix |
|---------|------------------|-----|
| Spins in place when commanded forward | FL/FR/BL/BR module order is wrong | Swap module assignments in `frontLeftIds` etc. |
| Drifts left/right when commanded forward | One module is inverted | Check drive motor inversion in TunerX |
| Robot overshoots or oscillates | Steer PID gains wrong | Re-run TunerX self-test or manually tune kP, kI, kD |

### Build errors

| Error message | Fix |
|---------------|-----|
| `cannot find symbol: ModuleIOTalonFXReal` | Add the import line listed in Step 6a |
| `UnsupportedOperationException: RealRobotProfile.createDrive() not yet implemented` | You deployed with `isReal()` path still throwing. Complete Step 6. |
| `IllegalArgumentException: wheelRadius must be > 0` | Your `wheelRadius` value is zero or negative |
| `IllegalArgumentException: bumperLength must be >= 0.5 m` | Your bumper measurement is under 19.7 in — double-check and convert to inches |

### CAN device errors on Driver Station

| Error | Fix |
|-------|-----|
| `TalonFX ID X: No device found` | Wrong CAN ID in `SwerveConstants` — verify in Phoenix Tuner X |
| `Pigeon 2 not connected` | Wrong `gyroCanId`, or Pigeon is on a different CAN bus than `canBusName` |
| `SparkMAX ID X: No device found` | Wrong CAN ID; verify in REV Hardware Client |

---

## Where to go next

| Goal | Resource |
|------|---------|
| All `SwerveConstants` fields and valid ranges | [Configuration](configuration.md) |
| Simulation tools, log replay, AdvantageScope tips | [Simulation](simulation.md) |
| Running automated tests | [Testing](testing.md) |
| Vision cameras (PhotonVision / Limelight) | [Robot Profiles → Vision](robot-profiles.md) or `/new-vision-camera` |
| Understanding how the IO layers work | [Architecture](architecture.md) |
