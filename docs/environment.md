# Environments — local, Codespaces, Claude Code on the web, CI

The library runs in four places. Most of the time you don't need to think about it — the same JDK 17, the same Gradle wrapper, the same vendordeps. This page only covers the per-environment configuration that's not in the code.

---

## Local — Windows

Primary workspace: `C:\workspace\FRC5010Claude`. Invoke Gradle from PowerShell:

```powershell
.\gradlew.bat test
.\gradlew.bat simulateJava
```

WSL has no access to `C:\workspace` (it's not mounted under `/mnt/c` by default), so `./gradlew` via Bash fails. Slash commands in `.claude/commands/` are written for this workflow.

## Local — Linux / macOS

Same project, different wrapper script:

```bash
./gradlew test
./gradlew simulateJava
```

The `gradlew` script is already executable in-repo. Translate `.\gradlew.bat` → `./gradlew` when following any slash command.

## Codespaces / devcontainer

`.devcontainer/` provides Java 17 on Debian Bookworm plus `xvfb` so `simulateJava` works without a display:

```bash
xvfb-run ./gradlew simulateJava
```

`postCreateCommand` pre-warms Gradle so the first test run is fast. Ports forwarded: **5810** (NT4 — AdvantageScope connects live to this), **5800** (web UI when `-PwebUI` is set), **1735** (legacy NT3).

## CI

`.github/workflows/ci.yml` runs `./gradlew test` on every push/PR to `main`. Layers 1–3 only — Layer 4 (`simulateJava`) is never in CI.

---

## Claude Code on the web (claude.ai/code)

To use this repo in a [claude.ai/code](https://claude.ai/code) web session:

1. **Install the Claude Code GitHub App** — go to [github.com/apps/claude](https://github.com/apps/claude), click **Install**, and grant it read/write access to this repository. Without write access the agent cannot push commits or create branches.
2. **Allow the required domains** — when creating a new environment, set the **network policy** to allow the domains listed below so the Gradle build and vendordep downloads succeed. See the [environment configuration docs](https://code.claude.com/docs/en/claude-code-on-the-web) for how to set the allowed-domains list.

### Trusted domains

| Domain | Purpose |
|--------|---------|
| `services.gradle.org` | Gradle wrapper distribution (`gradle-8.11-bin.zip`) |
| `plugins.gradle.org` | Gradle Plugin Portal — GradleRIO plugin |
| `frcmaven.wpi.edu` | WPILib Maven — WPILib libraries + AdvantageKit |
| `repo1.maven.org` | Maven Central — JUnit, YAGSL transitive deps |
| `maven.ctr-electronics.com` | CTRE Phoenix 6 |
| `maven.revrobotics.com` | REV Robotics |
| `maven.reduxrobotics.com` | Redux Robotics |
| `docs.home.thethriftybot.com` | ThriftyBot library |
| `maven.photonvision.org` | PhotonVision |
| `yet-another-software-suite.github.io` | YAGSL |
| `3015rangerrobotics.github.io` | PathPlannerLib |
| `jitpack.io` | JitPack — BLine-Lib path-following library (`com.github.edanliahovetsky:BLine-Lib`) |
| `pypi.org` | Python packages for the `frc-docs` MCP server (`uvx first-agentic-csa`) |
| `files.pythonhosted.org` | Python package downloads for `frc-docs` MCP server |
