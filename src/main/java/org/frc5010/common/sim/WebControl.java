package org.frc5010.common.sim;

import java.util.function.BooleanSupplier;
import java.util.function.IntSupplier;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import org.frc5010.common.drive.swerve.akit.AkitSwerveDrive;
import org.frc5010.common.input.WebXboxController;

/**
 * Encapsulates the full web UI control lifecycle: starts the {@link WebDriveController}
 * HTTP server, injects web button suppliers into a {@link WebXboxController}, and schedules
 * the {@code applyPendingControl} loop.
 *
 * <p>Only constructed when {@code -PwebUI} is set. All web-specific logic lives here;
 * {@link org.frc5010.common.profiles.SwerveRobotContainer} holds a nullable reference and
 * checks it before delegating.
 */
public class WebControl {

    private static WebControl instance;

    /**
     * Returns the active web UI instance, or empty when not running in web UI mode.
     * Valid after {@link org.frc5010.common.profiles.SwerveRobotContainer#configureBindings()}
     * completes — do not call during static initialisation.
     */
    public static java.util.Optional<WebControl> getInstance() {
        return java.util.Optional.ofNullable(instance);
    }

    private final WebDriveController webController;

    /**
     * Creates and starts the web UI server, injects web button suppliers into the
     * controller, and schedules the {@code applyPendingControl} loop.
     *
     * @param drive      the swerve drive subsystem
     * @param controller the web-aware Xbox controller to inject buttons into
     * @param resetPose  called when the alliance selection changes (re-anchors start pose)
     */
    public WebControl(AkitSwerveDrive drive, WebXboxController controller, Runnable resetPose) {
        instance = this;
        webController = new WebDriveController(drive);
        webController.start();

        BooleanSupplier[] webBtns = new BooleanSupplier[6];
        for (int i = 0; i < webBtns.length; i++) webBtns[i] = webController.getButton(i);
        controller.setWebInputs(webBtns);

        // Runs every cycle even while disabled so the Enable button in the web UI is processed.
        CommandScheduler.getInstance().schedule(
            Commands.run(() -> webController.applyPendingControl(resetPose))
                .ignoringDisable(true)
                .withName("WebControlApply"));
    }

    /** True if the browser sent a drive command within the connection window. */
    public boolean isConnected() { return webController.isConnected(); }

    /** True if connected but the last command has gone stale — caller should stop the robot. */
    public boolean isStale()     { return webController.isStale(); }

    /** Returns the current web-commanded chassis speeds, scaled to physical units. */
    public ChassisSpeeds getChassisSpeeds() { return webController.getChassisSpeeds(); }

    /**
     * Returns a {@link Trigger} for web button {@code idx}
     * (0=A, 1=B, 2=X, 3=Y, 4=LB, 5=RB).
     */
    public Trigger button(int idx) { return webController.buttonTrigger(idx); }

    /**
     * Binds demo-intake state suppliers so {@code /api/state} can surface them.
     * Suppliers are called from the HTTP thread — implementations must be thread-safe.
     *
     * @param heldFuel       number of fuel pieces currently held
     * @param intakeExtended whether the intake is extended
     * @param scoredFuel     number of fuel pieces scored
     */
    public void bindDemoState(IntSupplier heldFuel, BooleanSupplier intakeExtended,
                              IntSupplier scoredFuel) {
        webController.bindDemoState(heldFuel, intakeExtended, scoredFuel);
    }

    /**
     * Registers the available autonomous routines so the web UI can list and select them.
     * The selector and Auto/Teleop mode buttons in the web Driver Station panel drive this.
     *
     * @param names           ordered auto-routine names shown in the selector dropdown
     * @param initialSelected the name selected on first load (e.g. the default "None")
     * @param onSelect        called on the robot thread when the UI picks an auto
     */
    public void bindAutos(String[] names, String initialSelected,
                          java.util.function.Consumer<String> onSelect) {
        webController.bindAutos(names, initialSelected, onSelect);
    }
}
