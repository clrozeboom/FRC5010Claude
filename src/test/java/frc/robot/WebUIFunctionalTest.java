package frc.robot;

import static org.junit.jupiter.api.Assertions.*;

import java.io.BufferedReader;
import java.io.File;
import java.io.IOException;
import java.io.InputStreamReader;
import java.net.URI;
import java.net.http.HttpClient;
import java.net.http.HttpRequest;
import java.net.http.HttpResponse;
import java.time.Duration;
import java.util.Arrays;
import java.util.List;
import java.util.concurrent.TimeUnit;

import org.junit.jupiter.api.AfterAll;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.MethodOrderer;
import org.junit.jupiter.api.Order;
import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.TestMethodOrder;

/**
 * Layer 4 — functional tests that exercise the full robot stack via the
 * {@link org.frc5010.common.sim.WebDriveController} HTTP API.
 *
 * <p>{@code @BeforeAll} spawns {@code ./gradlew simulateJava -PwebUI -PtestSim --no-daemon}
 * as a subprocess and polls {@code /api/state} until the HTTP server responds (max 60 s).
 * {@code @AfterAll} sends {@code POST /api/stop} to trigger a clean exit.
 *
 * <p>Tests are ordered so each can rely on the state established by the previous one
 * (disabled → enabled → driving → intake extended). A single sim process serves all four tests.
 *
 * <p>Run with: {@code ./gradlew functionalTest}
 * <br>Report: {@code build/reports/tests/functionalTest/index.html}
 *
 * <p>This class is excluded from {@code ./gradlew test} via the {@code functional} JUnit tag.
 * Run it manually after major architecture changes, not on every commit.
 */
@Tag("functional")
@TestMethodOrder(MethodOrderer.OrderAnnotation.class)
class WebUIFunctionalTest {

    private static final String BASE_URL = "http://localhost:5800";
    private static final int    SIM_STARTUP_TIMEOUT_S = 90;

    private static Process      simProcess;
    private static HttpClient   httpClient;
    private static final StringBuilder simOutput = new StringBuilder();

    // ── Lifecycle ─────────────────────────────────────────────────────────────

    @BeforeAll
    static void startSim() throws Exception {
        httpClient = HttpClient.newBuilder()
            .connectTimeout(Duration.ofSeconds(3))
            .build();

        List<String> cmd = buildGradleCommand(
            "simulateJava", "-PwebUI", "-PtestSim", "--no-daemon");

        ProcessBuilder pb = new ProcessBuilder(cmd);
        pb.directory(new File(System.getProperty("user.dir")));
        pb.redirectErrorStream(true);
        simProcess = pb.start();

        // Drain subprocess output in background so the OS pipe buffer never fills.
        Thread drainer = new Thread(() -> {
            try (BufferedReader r = new BufferedReader(
                    new InputStreamReader(simProcess.getInputStream()))) {
                String line;
                while ((line = r.readLine()) != null) {
                    simOutput.append(line).append('\n');
                }
            } catch (IOException ignored) {}
        });
        drainer.setDaemon(true);
        drainer.start();

        // Poll /api/state until the HTTP server responds or we time out.
        HttpRequest probe = HttpRequest.newBuilder()
            .uri(URI.create(BASE_URL + "/api/state"))
            .timeout(Duration.ofSeconds(3))
            .GET().build();

        long deadline = System.currentTimeMillis() + SIM_STARTUP_TIMEOUT_S * 1_000L;
        while (System.currentTimeMillis() < deadline) {
            // Bail early if the subprocess died before the server came up.
            if (!simProcess.isAlive()) {
                throw new RuntimeException(
                    "Simulation process exited prematurely (exit code " +
                    simProcess.exitValue() + ").\nOutput:\n" + simOutput);
            }
            try {
                HttpResponse<String> resp =
                    httpClient.send(probe, HttpResponse.BodyHandlers.ofString());
                if (resp.statusCode() == 200) {
                    System.out.println("[FunctionalTest] Sim ready. Initial state: " + resp.body());
                    return;
                }
            } catch (Exception ignored) {}
            Thread.sleep(1_000);
        }
        throw new RuntimeException(
            "Simulation did not respond on port 5800 within " + SIM_STARTUP_TIMEOUT_S +
            " s.\nOutput:\n" + simOutput);
    }

    @AfterAll
    static void stopSim() {
        try { post("/api/stop", "{}"); } catch (Exception ignored) {}
        if (simProcess != null) {
            try { simProcess.waitFor(10, TimeUnit.SECONDS); } catch (InterruptedException ignored) {}
            if (simProcess.isAlive()) simProcess.destroyForcibly();
        }
    }

    // ── Tests (ordered — each builds on the state left by the previous) ───────

    /**
     * Verifies the robot starts in the disabled state.
     * This is the baseline before any /api/control command is sent.
     */
    @Test @Order(1)
    void robotStartsDisabled() throws Exception {
        String state = get("/api/state");
        assertTrue(bodyContains(state, "enabled", "false"),
            "Robot must start disabled before any /api/control command. State: " + state);
    }

    /**
     * Sends {@code POST /api/control {"enabled":true}} and verifies the robot enables.
     * Exercises the full {@code WebDriveController.applyPendingControl()} →
     * {@code DriverStationSim.setEnabled()} → {@code AkitSwerveDrive.periodic()} chain.
     */
    @Test @Order(2)
    void robotEnablesViaHttpControl() throws Exception {
        post("/api/control", "{\"enabled\":true,\"alliance\":\"Blue\"}");
        // applyPendingControl() runs on the WebControlApply command (ignoringDisable=true)
        // every 20 ms loop, so 500 ms is more than enough for the enable to propagate.
        waitMs(500);
        String state = get("/api/state");
        assertTrue(bodyContains(state, "enabled", "true"),
            "Robot must be enabled after POST /api/control {enabled:true}. State: " + state);
    }

    /**
     * Sends a forward drive command (vx = 0.5 normalised → 50% of max linear speed)
     * and verifies the robot's X pose increases by at least 0.5 m after 1 second.
     * Exercises the drive default command, IronMaple physics, and the pose snapshot in
     * {@code WebDriveController.getChassisSpeeds()}.
     *
     * <p>Requires the robot to be enabled (established by {@link #robotEnablesViaHttpControl}).
     */
    @Test @Order(3)
    void driveCommandDisplacesRobotPoseX() throws Exception {
        // poseBuf in WebDriveController starts at [0,0,0] and is only written inside
        // the drive default command (which only fires once isConnected() is true, i.e.
        // after the first /api/drive POST).  Send a zero-velocity command to warm up
        // poseBuf before reading x0, so the baseline reflects the robot's actual pose.
        post("/api/drive",
            "{\"vx\":0.0,\"vy\":0.0,\"omega\":0.0," +
            "\"buttons\":[false,false,false,false,false,false]}");
        waitMs(200);
        String state0 = get("/api/state");
        double x0 = extractDouble(state0, "x");

        // vx/vy/omega are normalised -1..1; getChassisSpeeds() multiplies by maxLinearMps.
        // A single POST sets commandBuf; isConnected() stays true for 5 s so the drive
        // default command reads the same command every loop until it becomes stale.
        post("/api/drive",
            "{\"vx\":0.5,\"vy\":0.0,\"omega\":0.0," +
            "\"buttons\":[false,false,false,false,false,false]}");
        waitMs(1_000);

        String state1 = get("/api/state");
        double x1 = extractDouble(state1, "x");

        assertTrue(x1 - x0 > 0.5,
            String.format("Forward drive (vx=0.5) for 1 s must displace X by > 0.5 m. " +
                "Got Δx = %.3f m (x0=%.3f, x1=%.3f). State: %s", x1 - x0, x0, x1, state1));
    }

    /**
     * Presses the LB web button (index 4) via a drive POST and verifies
     * {@code DemoIntake.intakeExtended} flips to {@code true} in the next state poll.
     * Exercises the DemoIntake default command and the {@code bindDemoState()} supplier
     * chain through to {@code /api/state}.
     *
     * <p>Requires the robot to be enabled (established by {@link #robotEnablesViaHttpControl}).
     */
    @Test @Order(4)
    void demoIntakeExtendsViaWebButton() throws Exception {
        // LB = buttons index 4.  A rising edge in DemoIntake.step() sets intakeExtended=true.
        post("/api/drive",
            "{\"vx\":0.0,\"vy\":0.0,\"omega\":0.0," +
            "\"buttons\":[false,false,false,false,true,false]}");
        // DemoIntake default command runs every 20 ms while enabled; 200 ms = 10 loops.
        waitMs(200);
        String state = get("/api/state");
        assertTrue(bodyContains(state, "intakeExtended", "true"),
            "DemoIntake must extend when LB (buttons[4]=true) is received. State: " + state);
    }

    /**
     * Verifies {@code GET /api/autos} lists the registered routines (including the default
     * "None") and reports a selected routine. Read-only — does not change robot state.
     */
    @Test @Order(5)
    void autosEndpointListsRoutines() throws Exception {
        String autos = get("/api/autos");
        assertTrue(autos.contains("\"None\""),
            "Auto list must contain the default 'None' option. Body: " + autos);
        assertTrue(autos.contains("BLine: Example Score (JSON)"),
            "Auto list must contain the JSON example routine. Body: " + autos);
        assertTrue(autos.contains("\"selected\":"),
            "Auto list must report the selected routine. Body: " + autos);
    }

    /**
     * Selects the JSON example routine, switches to autonomous mode, and enables. Verifies the
     * selection is reflected in {@code /api/autos}, the state reports {@code mode:auto}, and
     * BLine actually drives the robot toward the routine's goal pose (~3.0, 2.0). Exercises the
     * full auto-selector → mode → {@code getAutonomousCommand()} → {@code autonomousInit()} chain.
     */
    @Test @Order(6)
    void autoModeRunsSelectedRoutine() throws Exception {
        // Disable first so the mode switch takes effect on the next enable (real-DS semantics).
        post("/api/control", "{\"enabled\":false}");
        waitMs(300);
        post("/api/control", "{\"auto\":\"BLine: Example Score (JSON)\"}");
        post("/api/control", "{\"mode\":\"auto\"}");
        waitMs(300);

        String autos = get("/api/autos");
        assertTrue(autos.contains("\"selected\":\"BLine: Example Score (JSON)\""),
            "Selecting an auto must update /api/autos selected. Body: " + autos);

        // Enable in autonomous — the robot resets to start, then BLine drives to (3.0, 2.0).
        post("/api/control", "{\"enabled\":true}");
        waitMs(3_000);

        String state = get("/api/state");
        assertTrue(state.contains("\"mode\":\"auto\""),
            "State must report autonomous mode after enabling in auto. State: " + state);
        double x = extractDouble(state, "x");
        double y = extractDouble(state, "y");
        assertTrue(x > 2.5 && Math.abs(y - 2.0) < 0.5,
            String.format("ExampleScore auto must drive the robot to ~(3.0, 2.0); got (%.2f, %.2f). "
                + "State: %s", x, y, state));
    }

    // ── HTTP helpers ──────────────────────────────────────────────────────────

    private static String get(String path) throws Exception {
        HttpResponse<String> resp = httpClient.send(
            HttpRequest.newBuilder()
                .uri(URI.create(BASE_URL + path))
                .timeout(Duration.ofSeconds(5))
                .GET().build(),
            HttpResponse.BodyHandlers.ofString());
        return resp.body();
    }

    private static String post(String path, String json) throws Exception {
        HttpResponse<String> resp = httpClient.send(
            HttpRequest.newBuilder()
                .uri(URI.create(BASE_URL + path))
                .timeout(Duration.ofSeconds(5))
                .header("Content-Type", "application/json")
                .POST(HttpRequest.BodyPublishers.ofString(json)).build(),
            HttpResponse.BodyHandlers.ofString());
        return resp.body();
    }

    /** Returns true if the JSON body contains {@code "key":value} as a literal substring. */
    private static boolean bodyContains(String body, String key, String value) {
        return body.contains("\"" + key + "\":" + value);
    }

    /** Extracts a numeric value for {@code key} from a flat JSON object string. */
    private static double extractDouble(String body, String key) {
        int idx = body.indexOf("\"" + key + "\":");
        if (idx < 0) throw new RuntimeException("Key not found in JSON: " + key + "  body=" + body);
        idx += key.length() + 3;  // skip "key":
        int end = idx;
        while (end < body.length()) {
            char c = body.charAt(end);
            if (Character.isDigit(c) || c == '.' || c == '-' || c == 'E' || c == 'e') end++;
            else break;
        }
        return Double.parseDouble(body.substring(idx, end));
    }

    private static void waitMs(long ms) throws InterruptedException {
        Thread.sleep(ms);
    }

    /**
     * Builds the Gradle wrapper command appropriate for the current OS.
     * On Windows the wrapper is {@code gradlew.bat} and must be run via {@code cmd /c};
     * on Linux/macOS it is {@code ./gradlew}.
     */
    private static List<String> buildGradleCommand(String... tasks) {
        boolean windows = System.getProperty("os.name", "").toLowerCase().contains("windows");
        if (windows) {
            String[] full = new String[tasks.length + 3];
            full[0] = "cmd"; full[1] = "/c"; full[2] = "gradlew.bat";
            System.arraycopy(tasks, 0, full, 3, tasks.length);
            return Arrays.asList(full);
        }
        String[] full = new String[tasks.length + 1];
        full[0] = "./gradlew";
        System.arraycopy(tasks, 0, full, 1, tasks.length);
        return Arrays.asList(full);
    }
}
