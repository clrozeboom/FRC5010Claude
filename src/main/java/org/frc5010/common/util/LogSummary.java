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
 * }</pre>
 *
 * <p>Output sections:
 * <ol>
 *   <li>Header — file path and duration</li>
 *   <li>All entries — every signal name and type logged in the file</li>
 *   <li>Numeric statistics — min/max for every double-typed signal</li>
 *   <li>Anomaly flags — loop overruns, disconnected gyro, excessive currents</li>
 * </ol>
 */
public class LogSummary {

  // Current threshold above which a drive motor current is flagged.
  private static final double CURRENT_OVERLOAD_AMPS = 60.0;
  // Loop cycle time in microseconds above which an overrun is flagged.
  private static final long LOOP_OVERRUN_US = 25_000;

  public static void main(String[] args) throws Exception {
    String path = (args.length > 0) ? args[0] : findLatestLog();
    if (path == null) {
      System.err.println("No .wpilog file found.");
      System.err.println("Run the simulation first: .\\gradlew.bat simulateJava");
      System.exit(1);
    }

    System.out.println("=== Log Summary: " + path + " ===");
    System.out.println();
    analyzeLog(path);
  }

  // ---------------------------------------------------------------------------
  // Core analysis
  // ---------------------------------------------------------------------------

  private static void analyzeLog(String path) throws Exception {
    DataLogReader reader = new DataLogReader(path);

    Map<Integer, String> names = new HashMap<>();
    Map<Integer, String> types = new HashMap<>();
    Map<String, Double> mins   = new TreeMap<>();
    Map<String, Double> maxes  = new TreeMap<>();
    Map<String, Long>   counts = new TreeMap<>();
    long firstTs = Long.MAX_VALUE;
    long lastTs  = 0;
    long overrunCount = 0;

    for (DataLogRecord rec : reader) {
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

          // Detect loop overruns via the SystemStats epoch timestamp delta
          // (AdvantageKit records actual loop time under this key)
          if (name.contains("SystemStats") && name.contains("EpochTimeMicros") && v > LOOP_OVERRUN_US) {
            overrunCount++;
          }
        }
      } catch (Exception ignored) {
        // record type mismatch — skip
      }
    }

    double durationSec = (lastTs == 0) ? 0 : (lastTs - firstTs) / 1_000_000.0;
    System.out.printf("Duration : %.2f s%n", durationSec);
    System.out.printf("Entries  : %d%n", names.size());
    System.out.println();

    // --- All entries ---
    System.out.println("=== All Entries ===");
    new TreeMap<>(names).forEach((id, name) ->
        System.out.printf("  %-60s  (%s)%n", name, types.getOrDefault(id, "?")));
    System.out.println();

    // --- Numeric statistics ---
    System.out.println("=== Numeric Statistics (min / max) ===");
    mins.forEach((name, min) -> {
      double max = maxes.getOrDefault(name, min);
      long   n   = counts.getOrDefault(name, 0L);
      System.out.printf("  %-60s  %10.4f  /  %10.4f   (n=%d)%n",
          truncate(name, 60), min, max, n);
    });
    System.out.println();

    // --- Anomaly flags ---
    System.out.println("=== Anomaly Flags ===");
    boolean clean = true;

    if (overrunCount > 0) {
      System.out.printf("  [WARN] Loop overruns detected: %d cycles > %d ms%n",
          overrunCount, LOOP_OVERRUN_US / 1000);
      clean = false;
    }

    mins.forEach((name, min) -> {
      double max = maxes.getOrDefault(name, min);
      if (name.contains("CurrentAmps") && max > CURRENT_OVERLOAD_AMPS) {
        System.out.printf("  [WARN] High current: %s  max=%.1f A%n", name, max);
      }
    });

    // Flag gyro disconnected
    if (mins.containsKey("RealOutputs/Drive/GyroConnected")) {
      double minConnected = mins.get("RealOutputs/Drive/GyroConnected");
      if (minConnected < 0.5) {
        System.out.println("  [WARN] Gyro disconnected during log");
        clean = false;
      }
    }

    if (clean) {
      System.out.println("  No anomalies detected.");
    }
  }

  // ---------------------------------------------------------------------------
  // Helpers
  // ---------------------------------------------------------------------------

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
