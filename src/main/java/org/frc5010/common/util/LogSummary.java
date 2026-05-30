package org.frc5010.common.util;

import edu.wpi.first.util.datalog.DataLogReader;
import edu.wpi.first.util.datalog.DataLogRecord;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.Comparator;
import java.util.HashMap;
import java.util.Map;
import java.util.TreeMap;

/**
 * Command-line utility that reads a .wpilog file and prints a structured summary
 * of key swerve drive signals for agent-driven diagnosis and performance analysis.
 *
 * <p>Usage:
 * <pre>{@code
 * # Analyze the most recent log in logs/
 * .\gradlew.bat logSummary
 *
 * # Analyze a specific log
 * .\gradlew.bat logSummary -PlogFile=logs/FRC_20260525_143022.wpilog
 *
 * # Analyze the most recent replay log
 * .\gradlew.bat replayValidate
 * }</pre>
 *
 * <p>Output sections:
 * <ol>
 *   <li>Header — file path, log type (live/replay), and duration</li>
 *   <li>All entries — every signal name and type logged in the file</li>
 *   <li>Numeric statistics — min/max for every double-typed signal</li>
 *   <li>Anomaly flags — loop overruns, gyro/camera disconnects, excessive currents,
 *       vision observations all-rejected</li>
 * </ol>
 */
public class LogSummary {

  private static final double CURRENT_OVERLOAD_AMPS = 60.0;
  private static final double LOOP_OVERRUN_MS = 25.0;
  // Pose3d struct: Translation3d (3×8 bytes) + Rotation3d quaternion (4×8 bytes) = 56 bytes
  private static final int POSE3D_BYTES = 56;

  public static void main(String[] args) throws Exception {
    String path = (args.length > 0) ? args[0] : findLatestLog();
    if (path == null) {
      System.err.println("No .wpilog file found.");
      System.err.println("Run the simulation first: .\\gradlew.bat simulateJava");
      System.exit(1);
    }

    boolean isReplay = path.endsWith("_sim.wpilog");
    System.out.println("=== Log Summary" + (isReplay ? " [REPLAY]" : "") + ": " + path + " ===");
    System.out.println();
    analyzeLog(path, isReplay);
  }

  // ---------------------------------------------------------------------------
  // Core analysis
  // ---------------------------------------------------------------------------

  private static void analyzeLog(String path, boolean isReplay) throws Exception {
    DataLogReader reader = new DataLogReader(path);

    Map<Integer, String> names = new HashMap<>();
    Map<Integer, String> types = new HashMap<>();
    Map<String, Double>  mins   = new TreeMap<>();
    Map<String, Double>  maxes  = new TreeMap<>();
    Map<String, Long>    counts = new TreeMap<>();
    // Gyro signal names that went false (disconnected) at any point.
    java.util.Set<String> gyroDisconnected = new java.util.LinkedHashSet<>();
    // Vision: max raw bytes seen per Accepted/Rejected pose-array entry (0 = always empty).
    Map<String, Integer> visionPoseMaxRaw = new TreeMap<>();
    java.util.Set<String> visionCamsDisconnected = new java.util.LinkedHashSet<>();

    long firstTs = Long.MAX_VALUE;
    long lastTs  = 0;

    // A log produced by a sim that was killed mid-write ends in a truncated record.
    // The iterator throws when it tries to read past EOF; catch it and report what we
    // managed to parse rather than failing the whole analysis.
    java.util.Iterator<DataLogRecord> it = reader.iterator();
    while (true) {
      DataLogRecord rec;
      try {
        if (!it.hasNext()) break;
        rec = it.next();
      } catch (Exception truncated) {
        System.out.println("  (note: log ends in a truncated record — "
            + "likely a sim that was killed mid-write; reporting parsed data so far)");
        break;
      }
      long ts = rec.getTimestamp();
      if (ts < firstTs) firstTs = ts;
      if (ts > lastTs)  lastTs  = ts;

      if (rec.isStart()) {
        var d = rec.getStartData();
        names.put(d.entry, d.name);
        types.put(d.entry, d.type);
        continue;
      }
      if (rec.isControl()) continue;

      String name = names.getOrDefault(rec.getEntry(), "?entry" + rec.getEntry());
      String type = types.getOrDefault(rec.getEntry(), "");

      try {
        if ("double".equals(type)) {
          double v = rec.getDouble();
          mins.merge(name, v, Math::min);
          maxes.merge(name, v, Math::max);
          counts.merge(name, 1L, Long::sum);
        } else if ("boolean".equals(type) && name.contains("Gyro") && name.endsWith("Connected")) {
          if (!rec.getBoolean()) gyroDisconnected.add(name);
        } else if (isVisionEntry(name)) {
          if (isPoseArray(type) && (name.contains("Accepted") || name.contains("Rejected"))) {
            // getRaw() returns the raw struct bytes; Pose3d[] has 0 bytes when empty,
            // N×56 bytes when non-empty (no length prefix in WPILib struct encoding).
            int rawLen = rec.getRaw().length;
            visionPoseMaxRaw.merge(name, rawLen, Math::max);
          } else if ("boolean".equals(type) && name.endsWith("Connected")) {
            if (!rec.getBoolean()) visionCamsDisconnected.add(name);
          }
        }
      } catch (Exception ignored) {
        // record type mismatch — skip
      }
    }

    double durationSec = (lastTs == 0) ? 0 : (lastTs - firstTs) / 1_000_000.0;
    System.out.printf("Duration : %.2f s%n", durationSec);
    System.out.printf("Entries  : %d%n", names.size());
    if (isReplay) System.out.println("Log type : REPLAY (_sim.wpilog)");
    System.out.println();

    // --- All entries ---
    System.out.println("=== All Entries ===");
    new TreeMap<>(names).forEach((id, n) ->
        System.out.printf("  %-60s  (%s)%n", n, types.getOrDefault(id, "?")));
    System.out.println();

    // --- Numeric statistics ---
    System.out.println("=== Numeric Statistics (min / max) ===");
    mins.forEach((n, min) -> {
      double max = maxes.getOrDefault(n, min);
      long   cnt = counts.getOrDefault(n, 0L);
      System.out.printf("  %-60s  %10.4f  /  %10.4f   (n=%d)%n",
          truncate(n, 60), min, max, cnt);
    });
    System.out.println();

    // --- Anomaly flags ---
    System.out.println("=== Anomaly Flags ===");
    boolean[] found = {false};

    // Loop overrun: AdvantageKit records actual wall time per cycle in FullCycleMS.
    double maxCycleMs = maxes.getOrDefault("/RealOutputs/LoggedRobot/FullCycleMS", 0.0);
    if (maxCycleMs > LOOP_OVERRUN_MS) {
      System.out.printf("  [WARN] Loop overrun: FullCycleMS max=%.1f ms (threshold %.0f ms)%n",
          maxCycleMs, LOOP_OVERRUN_MS);
      found[0] = true;
    }

    mins.forEach((n, min) -> {
      double max = maxes.getOrDefault(n, min);
      if (n.contains("CurrentAmps") && max > CURRENT_OVERLOAD_AMPS) {
        System.out.printf("  [WARN] High current: %s  max=%.1f A%n", n, max);
        found[0] = true;
      }
    });

    gyroDisconnected.forEach(n -> {
      System.out.printf("  [WARN] Gyro disconnected during log: %s%n", n);
      found[0] = true;
    });

    // Vision anomalies
    if (!visionPoseMaxRaw.isEmpty() || !visionCamsDisconnected.isEmpty()) {
      System.out.println();
      System.out.println("  --- Vision ---");

      visionCamsDisconnected.forEach(n -> {
        System.out.printf("  [WARN] Camera disconnected: %s%n", n);
        found[0] = true;
      });

      boolean hasAcceptedKeys = visionPoseMaxRaw.keySet().stream()
          .anyMatch(k -> k.contains("Accepted") && !k.contains("Summary"));
      boolean anyAccepted = visionPoseMaxRaw.entrySet().stream()
          .anyMatch(e -> e.getKey().contains("Accepted") && !e.getKey().contains("Summary")
              && e.getValue() > 0);
      boolean anyRejected = visionPoseMaxRaw.entrySet().stream()
          .anyMatch(e -> e.getKey().contains("Rejected") && e.getValue() > 0);

      if (hasAcceptedKeys && !anyAccepted && anyRejected) {
        System.out.println("  [WARN] Vision: observations detected but all rejected "
            + "— check ambiguity/field-bounds filters");
        found[0] = true;
      }

      visionPoseMaxRaw.forEach((n, maxRaw) -> {
        int maxPoses = (maxRaw > 0) ? Math.max(1, maxRaw / POSE3D_BYTES) : 0;
        System.out.printf("  [INFO] %-58s  max_poses/frame=%d%n", truncate(n, 58), maxPoses);
      });
    }

    if (!found[0]) {
      System.out.println("  No anomalies detected.");
    }
  }

  // ---------------------------------------------------------------------------
  // Helpers
  // ---------------------------------------------------------------------------

  private static boolean isVisionEntry(String name) {
    return name.contains("/Vision/");
  }

  private static boolean isPoseArray(String type) {
    // AdvantageKit generates "struct:Pose3d[]" for Pose3d[] @AutoLog fields.
    return type.contains("Pose3d") && type.contains("[]");
  }

  private static String findLatestLog() throws Exception {
    Path dir = Paths.get("logs");
    if (!Files.isDirectory(dir)) return null;
    return Files.list(dir)
        .filter(p -> p.toString().endsWith(".wpilog"))
        .max(Comparator.comparingLong(p -> p.toFile().lastModified()))
        .map(Path::toString)
        .orElse(null);
  }

  private static String truncate(String s, int max) {
    return s.length() <= max ? s : "..." + s.substring(s.length() - (max - 3));
  }
}
