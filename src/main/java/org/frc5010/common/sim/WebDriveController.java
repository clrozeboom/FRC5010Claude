package org.frc5010.common.sim;

import com.sun.net.httpserver.HttpExchange;
import com.sun.net.httpserver.HttpServer;
import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import org.frc5010.common.drive.swerve.akit.AkitSwerveDrive;
import swervelib.simulation.ironmaple.simulation.SimulatedArena;
import swervelib.simulation.ironmaple.simulation.gamepieces.GamePieceOnFieldSimulation;
import swervelib.simulation.ironmaple.simulation.gamepieces.GamePieceProjectile;

import java.io.IOException;
import java.io.InputStream;
import java.io.OutputStream;
import java.net.BindException;
import java.net.InetSocketAddress;
import java.nio.charset.StandardCharsets;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicLong;
import java.util.concurrent.atomic.AtomicReference;
import java.util.function.BooleanSupplier;
import java.util.function.IntSupplier;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;

/**
 * Serves a browser-based field visualization and virtual controller on port 5800.
 * Only active in simulation ({@code RobotBase.isSimulation()} must be true before creating).
 *
 * <p>Thread model: HTTP handler threads only touch AtomicXxx fields. All WPILib API calls
 * happen on the robot thread via {@link #getChassisSpeeds()} and {@link #applyPendingControl}.
 */
public class WebDriveController {

    private static final int PORT = 5800;
    // Physics stalls (obstacle collisions) can cause loop overruns up to ~500 ms.
    // Use 1500 ms before flagging stale so transient stalls don't stop the robot.
    private static final long WATCHDOG_MS = 1500;
    private static final long CONNECTED_MS = 5000;

    private final AkitSwerveDrive drive;
    private final double maxLinearMps;
    private final double maxAngularRps;

    // Written by robot thread, read by HTTP thread (state endpoint)
    private final AtomicReference<double[]> poseBuf = new AtomicReference<>(new double[3]);
    private final AtomicBoolean enabledBuf = new AtomicBoolean(false);
    private final AtomicReference<String> allianceBuf = new AtomicReference<>("Blue");

    // Written by HTTP thread, read by robot thread (drive command)
    private final AtomicReference<double[]> commandBuf = new AtomicReference<>(new double[3]);
    private final AtomicBoolean[] buttons = new AtomicBoolean[6]; // A,B,X,Y,LB,RB
    private final AtomicLong lastCommandMs = new AtomicLong(0);

    // Pull-style demo state suppliers — wired by {@link #bindDemoState} if a demo intake exists.
    // Defaults return zero/false so the /api/state JSON always has the fields populated.
    // Suppliers are read from the HTTP thread; implementations must be thread-safe (volatile or atomic).
    private volatile IntSupplier heldFuelSupplier      = () -> 0;
    private volatile BooleanSupplier intakeExtendedSupplier = () -> false;
    private volatile IntSupplier scoredFuelSupplier    = () -> 0;

    // Pending DriverStation control — written by HTTP, applied on robot thread.
    // Nullable: null means "not set in this POST" so the robot thread skips that field.
    private final AtomicReference<Boolean> pendingEnabled = new AtomicReference<>(null);
    private final AtomicReference<String> pendingAlliance = new AtomicReference<>(null);
    private final AtomicReference<String> pendingMode = new AtomicReference<>(null);
    private final AtomicReference<String> pendingAutoSelect = new AtomicReference<>(null);
    private final AtomicBoolean controlPending = new AtomicBoolean(false);

    // Auto/teleop mode and auto-routine selection.
    // modeBuf / selectedAutoBuf are the applied state surfaced to the UI via /api/state.
    private final AtomicReference<String> modeBuf = new AtomicReference<>("teleop");
    private volatile String[] autoNames = new String[0];
    private final AtomicReference<String> selectedAutoBuf = new AtomicReference<>("");
    // Invoked on the robot thread when the UI picks an auto; the robot uses it to choose
    // which command getAutonomousCommand() returns. Null until bindAutos() wires it.
    private volatile java.util.function.Consumer<String> autoSelectCallback = null;

    private HttpServer server;
    private ExecutorService executor;

    public WebDriveController(AkitSwerveDrive drive) {
        this.drive = drive;
        this.maxLinearMps  = drive.getMaxLinearSpeed().in(MetersPerSecond);
        this.maxAngularRps = drive.getMaxAngularSpeed().in(RadiansPerSecond);
        for (int i = 0; i < buttons.length; i++) {
            buttons[i] = new AtomicBoolean(false);
        }
    }

    /** Starts the HTTP server. Silently skips if the port is already bound. */
    public void start() {
        try {
            executor = Executors.newFixedThreadPool(4);
            server = HttpServer.create(new InetSocketAddress(PORT), 0);
            server.createContext("/api/state",      this::handleState);
            server.createContext("/api/drive",      this::handleDrive);
            server.createContext("/api/control",    this::handleControl);
            server.createContext("/api/autos",      this::handleAutos);
            server.createContext("/api/gamepieces", this::handleGamePieces);
            server.createContext("/api/stop",       this::handleStop);
            server.createContext("/tags/",          this::handleTagImage);
            server.createContext("/fuel.png",       this::handleFuelImage);
            server.createContext("/",               this::handleRoot);
            server.setExecutor(executor);
            server.start();
            System.out.println("[WebDriveController] Robot web interface: http://localhost:" + PORT);
            SmartDashboard.putString("WebInterface/URL", "http://localhost:" + PORT);
        } catch (BindException e) {
            System.err.println("[WebDriveController] Warning: port " + PORT
                + " already in use — web interface disabled.");
        } catch (IOException e) {
            System.err.println("[WebDriveController] Warning: could not start HTTP server: "
                + e.getMessage());
        }
    }

    /** Graceful shutdown. */
    public void stop() {
        if (server   != null) server.stop(1);
        if (executor != null) executor.shutdownNow();
    }

    /**
     * Apply pending enable/alliance changes on the robot thread.
     * @param resetPose called when alliance changes to re-anchor the start pose
     */
    public void applyPendingControl(Runnable resetPose) {
        if (controlPending.compareAndSet(true, false)) {
            String alliance = pendingAlliance.getAndSet(null);
            if (alliance != null) {
                DriverStationSim.setAllianceStationId(
                    "Red".equals(alliance) ? AllianceStationID.Red1 : AllianceStationID.Blue1);
                allianceBuf.set(alliance);
                resetPose.run();
            }
            // Auto routine selection — hand the chosen name to the robot so its
            // getAutonomousCommand() returns the matching command.
            String auto = pendingAutoSelect.getAndSet(null);
            if (auto != null) {
                selectedAutoBuf.set(auto);
                if (autoSelectCallback != null) autoSelectCallback.accept(auto);
            }
            // Auto/teleop mode — stored now, applied to the DS autonomous bit on the next
            // enable (matching a real Driver Station, which can't switch mode while enabled).
            String mode = pendingMode.getAndSet(null);
            if (mode != null) modeBuf.set(mode);

            Boolean enabled = pendingEnabled.getAndSet(null);
            if (enabled != null) {
                // DriverStation.isEnabled() == controlWord.getEnabled() && getDSAttached().
                // Without DS-attached the enabled bit is ignored and the robot stays
                // disabled — AkitSwerveDrive.periodic() then stops every module. Mark the
                // (virtual) DS attached so the web Enable button actually takes effect.
                DriverStationSim.setDsAttached(true);
                DriverStationSim.setAutonomous("auto".equals(modeBuf.get()));
                DriverStationSim.setEnabled(enabled);
                DriverStationSim.setTest(false);
                enabledBuf.set(enabled);
            }
        }
        // Always notify every cycle so the sim DS treats the connection as alive.
        // Without periodic notifications some WPILib builds auto-disable after the
        // initial setEnabled() call ages out (matches real DS 20 ms packet rate).
        DriverStationSim.notifyNewData();
    }

    /**
     * Returns the current web-commanded chassis speeds, scaled to real m/s and rad/s.
     * Also snapshots the current pose into {@code poseBuf} for the state endpoint.
     * Must be called from the robot thread.
     */
    public ChassisSpeeds getChassisSpeeds() {
        Pose2d pose = drive.getPose();
        poseBuf.set(new double[]{pose.getX(), pose.getY(), pose.getRotation().getRadians()});
        double[] cmd = commandBuf.get();
        return new ChassisSpeeds(
            cmd[0] * maxLinearMps,
            cmd[1] * maxLinearMps,
            cmd[2] * maxAngularRps);
    }

    /** True if the browser sent a drive command within the last 2 s. */
    public boolean isConnected() { return age() < CONNECTED_MS; }

    /** True if connected but the last command is > 500 ms stale — stop the robot now. */
    public boolean isStale()     { return age() > WATCHDOG_MS && age() < CONNECTED_MS; }

    /**
     * Returns a supplier for button {@code idx} (0=A, 1=B, 2=X, 3=Y, 4=LB, 5=RB).
     * Safe to call from any thread.
     */
    public BooleanSupplier getButton(int idx) {
        if (idx < 0 || idx >= buttons.length) return () -> false;
        return buttons[idx]::get;
    }

    /** Wraps {@link #getButton(int)} in a WPILib {@link Trigger}. */
    public Trigger buttonTrigger(int idx) {
        return new Trigger(getButton(idx));
    }

    /**
     * Binds suppliers that the {@code /api/state} JSON reads each request to surface
     * demo-intake state in the web UI. Suppliers are invoked from the HTTP thread, so
     * implementations must be thread-safe (volatile or atomic backing fields).
     */
    public void bindDemoState(IntSupplier heldFuel, BooleanSupplier intakeExtended, IntSupplier scoredFuel) {
        this.heldFuelSupplier      = heldFuel;
        this.intakeExtendedSupplier = intakeExtended;
        this.scoredFuelSupplier    = scoredFuel;
    }

    /**
     * Registers the available autonomous routines so the web UI can list and select them.
     *
     * @param names           ordered auto-routine names shown in the selector dropdown
     * @param initialSelected the name selected on first load (e.g. the default "None")
     * @param onSelect        called on the robot thread when the UI picks an auto; the robot
     *                        uses the chosen name to decide what {@code getAutonomousCommand()}
     *                        returns. Must be safe to call from the robot thread.
     */
    public void bindAutos(String[] names, String initialSelected, java.util.function.Consumer<String> onSelect) {
        this.autoNames = names != null ? names : new String[0];
        this.selectedAutoBuf.set(initialSelected != null ? initialSelected : "");
        this.autoSelectCallback = onSelect;
    }

    private long age() { return System.currentTimeMillis() - lastCommandMs.get(); }

    // ---- HTTP handlers (all run on executor thread pool) ----

    private void handleRoot(HttpExchange ex) throws IOException {
        if (!"GET".equalsIgnoreCase(ex.getRequestMethod())) {
            ex.sendResponseHeaders(405, -1);
            return;
        }
        try (InputStream in = getClass().getResourceAsStream("/web/index.html")) {
            if (in == null) {
                respond(ex, 500, "text/plain", "index.html not found on classpath");
                return;
            }
            byte[] body = in.readAllBytes();
            addCors(ex);
            ex.getResponseHeaders().set("Content-Type", "text/html; charset=UTF-8");
            ex.sendResponseHeaders(200, body.length);
            try (OutputStream os = ex.getResponseBody()) { os.write(body); }
        }
    }

    /**
     * Returns a JSON array of current Fuel piece positions for the web field view.
     * Calls {@code gamePiecesOnField()} which is synchronized on the arena — safe
     * to call from this HTTP handler thread.
     */
    private void handleGamePieces(HttpExchange ex) throws IOException {
        addCors(ex);
        if (!"GET".equalsIgnoreCase(ex.getRequestMethod())) { ex.sendResponseHeaders(405, -1); return; }
        StringBuilder sb = new StringBuilder("{\"pieces\":[");
        boolean first = true;
        try {
            for (GamePieceOnFieldSimulation piece
                    : SimulatedArena.getInstance().gamePiecesOnField()) {
                if (!"Fuel".equals(piece.getType())) continue;
                var pose = piece.getPoseOnField();
                if (!first) sb.append(',');
                sb.append('[')
                  .append(String.format("%.3f", pose.getX())).append(',')
                  .append(String.format("%.3f", pose.getY())).append(']');
                first = false;
            }
        } catch (Exception ignored) {}
        sb.append("],\"flying\":[");
        boolean firstFly = true;
        try {
            for (GamePieceProjectile proj : SimulatedArena.getInstance().gamePieceLaunched()) {
                if (!"Fuel".equals(proj.getType())) continue;
                var p3 = proj.getPose3d();
                if (!firstFly) sb.append(',');
                sb.append('[')
                  .append(String.format("%.3f", p3.getX())).append(',')
                  .append(String.format("%.3f", p3.getY())).append(']');
                firstFly = false;
            }
        } catch (Exception ignored) {}
        sb.append("]}");
        respond(ex, 200, "application/json", sb.toString());
    }

    /** Serves the Fuel game-piece PNG ({@code /fuel.png}) from the classpath. */
    private void handleFuelImage(HttpExchange ex) throws IOException {
        addCors(ex);
        if (!"GET".equalsIgnoreCase(ex.getRequestMethod())) { ex.sendResponseHeaders(405, -1); return; }
        try (InputStream in = getClass().getResourceAsStream("/web/fuel.png")) {
            if (in == null) { ex.sendResponseHeaders(404, -1); return; }
            byte[] body = in.readAllBytes();
            ex.getResponseHeaders().set("Content-Type", "image/png");
            ex.getResponseHeaders().set("Cache-Control", "max-age=86400");
            ex.sendResponseHeaders(200, body.length);
            try (OutputStream os = ex.getResponseBody()) { os.write(body); }
        }
    }

    /** Serves AprilTag PNGs from the {@code /web/tags/} classpath resources. */
    private void handleTagImage(HttpExchange ex) throws IOException {
        addCors(ex);
        if (!"GET".equalsIgnoreCase(ex.getRequestMethod())) { ex.sendResponseHeaders(405, -1); return; }
        // Only allow simple "AT<n>.png" names — no path traversal.
        String name = ex.getRequestURI().getPath().substring("/tags/".length());
        if (!name.matches("AT\\d{1,2}\\.png")) { ex.sendResponseHeaders(404, -1); return; }
        try (InputStream in = getClass().getResourceAsStream("/web/tags/" + name)) {
            if (in == null) { ex.sendResponseHeaders(404, -1); return; }
            byte[] body = in.readAllBytes();
            ex.getResponseHeaders().set("Content-Type", "image/png");
            ex.getResponseHeaders().set("Cache-Control", "max-age=86400");
            ex.sendResponseHeaders(200, body.length);
            try (OutputStream os = ex.getResponseBody()) { os.write(body); }
        }
    }

    private void handleState(HttpExchange ex) throws IOException {
        addCors(ex);
        if ("OPTIONS".equalsIgnoreCase(ex.getRequestMethod())) { ex.sendResponseHeaders(204, -1); return; }
        double[] p  = poseBuf.get();
        String json = String.format(
            "{\"x\":%.4f,\"y\":%.4f,\"headingRad\":%.4f," +
            "\"maxLinear\":%.4f,\"maxAngular\":%.4f," +
            "\"fieldWidth\":16.540988,\"fieldHeight\":8.21," +
            "\"enabled\":%b,\"alliance\":\"%s\",\"connected\":%b," +
            "\"mode\":\"%s\",\"selectedAuto\":\"%s\"," +
            "\"heldFuel\":%d,\"intakeExtended\":%b,\"scoredFuel\":%d}",
            p[0], p[1], p[2], maxLinearMps, maxAngularRps,
            enabledBuf.get(), allianceBuf.get(), isConnected(),
            modeBuf.get(), jsonEscape(selectedAutoBuf.get()),
            heldFuelSupplier.getAsInt(), intakeExtendedSupplier.getAsBoolean(), scoredFuelSupplier.getAsInt());
        respond(ex, 200, "application/json", json);
    }

    /** Returns the registered auto-routine names and the currently selected one. */
    private void handleAutos(HttpExchange ex) throws IOException {
        addCors(ex);
        if ("OPTIONS".equalsIgnoreCase(ex.getRequestMethod())) { ex.sendResponseHeaders(204, -1); return; }
        if (!"GET".equalsIgnoreCase(ex.getRequestMethod()))    { ex.sendResponseHeaders(405, -1); return; }
        String[] names = autoNames;
        StringBuilder sb = new StringBuilder("{\"autos\":[");
        for (int i = 0; i < names.length; i++) {
            if (i > 0) sb.append(',');
            sb.append('"').append(jsonEscape(names[i])).append('"');
        }
        sb.append("],\"selected\":\"").append(jsonEscape(selectedAutoBuf.get())).append("\"}");
        respond(ex, 200, "application/json", sb.toString());
    }

    private void handleDrive(HttpExchange ex) throws IOException {
        addCors(ex);
        if ("OPTIONS".equalsIgnoreCase(ex.getRequestMethod())) { ex.sendResponseHeaders(204, -1); return; }
        if (!"POST".equalsIgnoreCase(ex.getRequestMethod()))   { ex.sendResponseHeaders(405, -1); return; }
        try {
            String body = new String(ex.getRequestBody().readAllBytes(), StandardCharsets.UTF_8);
            double vx    = clamp(extractDouble(body, "vx",    0.0));
            double vy    = clamp(extractDouble(body, "vy",    0.0));
            double omega = clamp(extractDouble(body, "omega", 0.0));
            commandBuf.set(new double[]{vx, vy, omega});
            lastCommandMs.set(System.currentTimeMillis());

            int arrStart = body.indexOf("\"buttons\":");
            if (arrStart >= 0) {
                arrStart = body.indexOf('[', arrStart);
                int arrEnd = body.indexOf(']', arrStart);
                if (arrStart >= 0 && arrEnd > arrStart) {
                    String[] parts = body.substring(arrStart + 1, arrEnd).split(",");
                    for (int i = 0; i < parts.length && i < buttons.length; i++) {
                        buttons[i].set("true".equals(parts[i].trim()));
                    }
                }
            }
        } catch (Exception ignored) {}
        respond(ex, 200, "application/json", "{}");
    }

    private void handleStop(HttpExchange ex) throws IOException {
        addCors(ex);
        if ("OPTIONS".equalsIgnoreCase(ex.getRequestMethod())) { ex.sendResponseHeaders(204, -1); return; }
        if (!"POST".equalsIgnoreCase(ex.getRequestMethod()))   { ex.sendResponseHeaders(405, -1); return; }
        respond(ex, 200, "application/json", "{\"stopping\":true}");
        new Thread(() -> {
            try { Thread.sleep(300); } catch (InterruptedException ignored) {}
            System.out.println("[WebDriveController] Stop requested from web interface.");
            System.exit(0);
        }, "web-stop").start();
    }

    private void handleControl(HttpExchange ex) throws IOException {
        addCors(ex);
        if ("OPTIONS".equalsIgnoreCase(ex.getRequestMethod())) { ex.sendResponseHeaders(204, -1); return; }
        if (!"POST".equalsIgnoreCase(ex.getRequestMethod()))   { ex.sendResponseHeaders(405, -1); return; }
        try {
            String body = new String(ex.getRequestBody().readAllBytes(), StandardCharsets.UTF_8);
            // Only update each field when it is present in the request body.
            if (body.contains("\"enabled\":")) {
                pendingEnabled.set(body.contains("\"enabled\":true"));
            }
            if      (body.contains("\"alliance\":\"Red\""))  pendingAlliance.set("Red");
            else if (body.contains("\"alliance\":\"Blue\"")) pendingAlliance.set("Blue");
            if      (body.contains("\"mode\":\"auto\""))     pendingMode.set("auto");
            else if (body.contains("\"mode\":\"teleop\""))   pendingMode.set("teleop");
            String auto = extractString(body, "auto");
            if (auto != null) pendingAutoSelect.set(auto);
            controlPending.set(true);
        } catch (Exception ignored) {}
        respond(ex, 200, "application/json", "{}");
    }

    // ---- Utilities ----

    private static void respond(HttpExchange ex, int code, String type, String body) throws IOException {
        byte[] bytes = body.getBytes(StandardCharsets.UTF_8);
        ex.getResponseHeaders().set("Content-Type", type + "; charset=UTF-8");
        ex.sendResponseHeaders(code, bytes.length);
        try (OutputStream os = ex.getResponseBody()) { os.write(bytes); }
    }

    private static void addCors(HttpExchange ex) {
        ex.getResponseHeaders().set("Access-Control-Allow-Origin",  "*");
        ex.getResponseHeaders().set("Access-Control-Allow-Methods", "GET, POST, OPTIONS");
        ex.getResponseHeaders().set("Access-Control-Allow-Headers", "Content-Type");
    }

    private static double extractDouble(String body, String key, double fallback) {
        int idx = body.indexOf("\"" + key + "\":");
        if (idx < 0) return fallback;
        idx += key.length() + 3;
        int end = idx;
        while (end < body.length()) {
            char c = body.charAt(end);
            if (Character.isDigit(c) || c == '.' || c == '-' || c == 'E' || c == 'e') end++;
            else break;
        }
        try { return Double.parseDouble(body.substring(idx, end)); }
        catch (NumberFormatException e) { return fallback; }
    }

    private static double clamp(double v) { return Math.max(-1.0, Math.min(1.0, v)); }

    /**
     * Extracts a JSON string value for {@code key} (i.e. {@code "key":"value"}). Returns null
     * when absent. Naive — does not decode escapes — which is sufficient for the simple
     * single-token payloads this server receives (auto names contain no quotes).
     */
    private static String extractString(String body, String key) {
        String needle = "\"" + key + "\":\"";
        int idx = body.indexOf(needle);
        if (idx < 0) return null;
        idx += needle.length();
        int end = body.indexOf('"', idx);
        if (end < 0) return null;
        return body.substring(idx, end);
    }

    /** Minimal JSON string escaping for values embedded in hand-built responses. */
    private static String jsonEscape(String s) {
        if (s == null) return "";
        return s.replace("\\", "\\\\").replace("\"", "\\\"");
    }
}
