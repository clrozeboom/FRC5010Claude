# Using GitHub Codespaces

GitHub Codespaces runs a full Linux development environment in a browser tab — no local install required beyond a GitHub account. Every dependency (Java 17, Gradle, vendordeps, `xvfb`) is pre-installed by the `.devcontainer`.

[![Open in GitHub Codespaces](https://github.com/codespaces/badge.svg)](https://codespaces.new/clrozeboom/FRC5010Claude)

> **This is the fastest way to get started.** If your computer can't install WPILib (school restrictions, wrong OS, etc.) or you just want to try the library before committing to a local setup, start here.

---

## Open the Codespace

1. Click the badge above, or go to your fork on GitHub and click **Code → Codespaces → Create codespace on main**.
2. A VS Code window opens in your browser. The first launch takes 3–5 minutes while it builds the container and pre-warms the Gradle cache (you'll see "Running postCreateCommand" in the terminal).
3. When the terminal prompt appears, the environment is ready.

> **No WPILib VS Code extension in Codespaces.** The WPILib extension is blocked from the VS Code Marketplace. Codespaces installs the standard Java/Gradle extensions instead. Build and run via the terminal — the WPILib palette commands (Ctrl+Shift+P → "WPILib:…") are not available.

---

## Build and test

```bash
./gradlew test
```

The test report appears at `build/reports/tests/test/index.html`. Right-click it in the VS Code file explorer and choose **Open with Live Server** (if you have the extension) or download it to view locally.

---

## Run the simulation (web UI mode — recommended for Codespaces)

```bash
./gradlew simulateJava -PwebUI
```

The devcontainer forwards port **5800** and is configured to open it automatically as a browser tab. Within a few seconds a new tab opens showing the field view with a virtual gamepad and Enable button.

**What you can do from the web UI:**
- Click **Enable** to enable the robot
- Use the on-screen joystick or keyboard arrow keys to drive
- Press **LB** to extend the intake and collect Fuel pieces
- Press **A** to fire held Fuel toward the hub
- Watch the robot and game pieces move on the 2D field
- Switch the alliance (Blue/Red) and watch the robot teleport to the correct start

No Glass, no AdvantageScope, no local software needed.

> **Port 5800 didn't open automatically?** In VS Code's **Ports** panel (bottom bar → "Ports"), find port 5800 and click the globe icon.

---

## Connect AdvantageScope for a 3D view (optional)

If you want the full 3D field view, AdvantageScope can connect to the forwarded NT4 port:

1. Run the simulation (any mode — `-PwebUI`, `-PvisualTest`, or plain):
   ```bash
   xvfb-run ./gradlew simulateJava -PvisualTest   # auto-enables and drives the visual test
   # OR (for interactive web UI):
   ./gradlew simulateJava -PwebUI
   ```
2. In VS Code's **Ports** panel, find port **5810** and hover over it — click the globe icon to copy the forwarded URL (looks like `https://<codespace-name>-5810.app.github.dev`).
3. In AdvantageScope on your local laptop: **File → Connect to Robot**. Paste the forwarded hostname only (strip the `https://` prefix and trailing `/`), leave port at `5810`.
4. Drag `RealOutputs/Drive/Pose` onto the 3D field canvas.

> **Why `xvfb-run` for `-PvisualTest`?** Glass tries to open a window. `xvfb-run` provides a virtual (invisible) display so it doesn't crash. You never see the Glass window — all observation is via AdvantageScope or the `.wpilog` file. With `-PwebUI`, Glass is skipped entirely so `xvfb-run` is not needed.

---

## Review a run offline (log replay)

Every simulation run writes a `.wpilog` to the `logs/` folder. To review it without a live connection:

1. In VS Code's file explorer, navigate to `logs/` and right-click the most recent `.wpilog` file → **Download**.
2. Open AdvantageScope on your local laptop → **File → Open Log** → select the downloaded file.
3. Scrub through the timeline, inspect poses, module states, and gyro readings.

---

## What Codespaces cannot do

| Task | Why it doesn't work in Codespaces | Workaround |
|------|------------------------------------|------------|
| Deploy to RoboRIO | No USB/network access to a physical robot | Use a local machine with WPILib installed |
| Glass GUI | No physical display; virtual display via `xvfb-run` works but Glass window is invisible | Use `-PwebUI` or observe via AdvantageScope + port 5810 |
| Phoenix Tuner X | Windows-only application | Run on a local Windows machine |
| Plug in a physical gamepad | No USB passthrough to the container | Use the on-screen web UI gamepad or keyboard |

---

## Saving your work

Codespaces are tied to your GitHub account. Changes you make are committed back to your fork the same way as on a local machine:

```bash
git add src/main/java/org/frc5010/examples/ExampleRobotProfile.java
git commit -m "Fill in robot measurements for 2026 season"
git push
```

> **Codespaces time out after 30 minutes of inactivity** (free tier). Uncommitted changes are not lost — the container is just paused. Your next visit resumes it. But commit regularly anyway.
