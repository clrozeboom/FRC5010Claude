# FRC5010Claude — Swerve Drive Framework

[![CI](https://github.com/clrozeboom/FRC5010Claude/actions/workflows/ci.yml/badge.svg)](https://github.com/clrozeboom/FRC5010Claude/actions/workflows/ci.yml)

A clean, AdvantageKit-compatible swerve drive library for FRC teams using WPILib 2026. Configure one `SwerveConstants` record and get a fully wired drivetrain subsystem that works identically on real hardware, in simulation, and in log replay — with zero mode-switching code in your robot.

---

## What you get

- **One config, three modes** — `SwerveFactory` selects real hardware IO, IronMaple physics simulation, or AdvantageKit replay automatically based on `RobotMode`.
- **WPILib units throughout** — all builder fields accept any unit (`Inches`, `Pounds`, `Hertz`, …); the library converts internally.
- **Robot profile pattern** — swap between a lightweight test robot and your real robot's parameters with a single Gradle flag.
- **Keyboard drive out of the box** — `SwerveRobotContainer` wires WASD keyboard drive, alliance-aware pose reset, and an automated visual test sequence for free.
- **Multi-layer test pyramid** — unit, subsystem-sim, and physics-integration layers; CI runs on every push.

---

## Quick start

1. **Extend `ExampleRobotProfile`** in `src/main/java/org/frc5010/examples/ExampleRobotProfile.java` and fill in your robot's measurements and CAN IDs (the file has TODO comments guiding you):

```java
private static final SwerveConstants CONSTANTS = new SwerveConstants.Builder()
    .moduleType(ModuleType.TALON_FX)
    .gyroType(GyroType.PIGEON2)
    .gyroCanId(1)
    .trackWidth(Inches.of(22.75))
    .wheelBase(Inches.of(22.75))
    .wheelRadius(Inches.of(2.0))
    .robotMass(Pounds.of(125))
    .bumperLength(Inches.of(30))
    .bumperWidth(Inches.of(30))
    .frontLeftIds(1, 2, 3)
    .frontRightIds(4, 5, 6)
    .backLeftIds(7, 8, 9)
    .backRightIds(10, 11, 12)
    .build();
```

2. **Run the simulation** to verify physics before touching hardware:

```powershell
.\gradlew.bat simulateJava        # Windows
./gradlew simulateJava            # macOS / Linux / Codespaces
```

3. **Wire real hardware IO** when you're ready to deploy — see [Robot Profiles](docs/robot-profiles.md).

---

## Documentation

| Topic | Link |
|-------|------|
| **New team? Start here** — fork, measure, configure, simulate, deploy | [Student Setup Guide](docs/student-setup.md) |
| **No local install?** — build, test, and drive from a browser tab | [GitHub Codespaces](docs/codespaces.md) |
| **Lessons** — guided, hands-on lessons + beginner concept pages | [Lessons hub](docs/lessons/README.md) |
| All `SwerveConstants` fields, defaults, and valid ranges | [Configuration](docs/configuration.md) |
| Simulation scenarios, Gradle flags, AdvantageScope | [Simulation](docs/simulation.md) |
| `RobotProfile` pattern, wiring hardware IO | [Robot Profiles](docs/robot-profiles.md) |
| Driver controls — input pipeline, button bindings, the `axis(2)` gotcha | [Controls](docs/controls.md) |
| Test pyramid, running tests, per-layer thresholds | [Testing](docs/testing.md) |
| IO layer diagram, per-cycle call order, factory internals | [Architecture](docs/architecture.md) |

---

## Vendordeps

| Library | Version | Purpose |
|---------|---------|---------|
| WPILib / GradleRIO | 2026.2.1 | Core robot framework |
| Phoenix 6 | 26.2.0 | TalonFX, CANcoder, Pigeon 2 |
| REVLib | latest | SparkMAX / SparkFlex |
| AdvantageKit | latest | IO abstraction + log replay |
| PathPlannerLib | 2026.1.2 | Autonomous path following |
| YAGSL / IronMaple | 2026.4.1 | Physics simulation engine |
| PhotonVision | latest | Vision targeting |

---

## Developing in GitHub Codespaces

[![Open in GitHub Codespaces](https://github.com/codespaces/badge.svg)](https://codespaces.new/clrozeboom/FRC5010Claude)

Full build, test, and headless simulation work out of the box — no local install required. The browser-based web UI (`-PwebUI`) opens automatically on port 5800 so you can drive and interact with the robot from any tab. See [docs/codespaces.md](docs/codespaces.md) for the full walkthrough.
