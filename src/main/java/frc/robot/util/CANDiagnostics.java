package frc.robot.util;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import edu.wpi.first.wpilibj.DriverStation;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import org.littletonrobotics.junction.Logger;

/**
 * Utility class for diagnosing CAN bus timeout issues with Phoenix 6 devices. Helps identify which
 * specific devices and signals are failing.
 */
public class CANDiagnostics {
  private static final Map<String, SignalHealth> signalHealthMap = new HashMap<>();
  private static final double HEALTH_CHECK_PERIOD = 2.0; // seconds
  private static double lastHealthCheckTime = 0.0;

  private static class SignalHealth {
    int successCount = 0;
    int timeoutCount = 0;
    int errorCount = 0;
    StatusCode lastStatus = StatusCode.OK;
    double lastErrorTime = 0.0;
  }

  /**
   * Check the status of a signal refresh operation and log any issues.
   *
   * @param subsystemName Name of the subsystem (e.g., "Elevator", "Intake")
   * @param signals Array of signals that were refreshed
   */
  public static void checkSignalHealth(String subsystemName, BaseStatusSignal... signals) {
    if (signals == null || signals.length == 0) return;

    List<String> failedSignals = new ArrayList<>();

    for (int i = 0; i < signals.length; i++) {
      BaseStatusSignal signal = signals[i];
      StatusCode status = signal.getStatus();
      String signalKey = subsystemName + "/" + signal.getName();

      // Get or create health tracker for this signal
      SignalHealth health = signalHealthMap.computeIfAbsent(signalKey, k -> new SignalHealth());
      health.lastStatus = status;

      // Track success/failure
      if (status == StatusCode.OK) {
        health.successCount++;
      } else if (!status.isOK()) {
        // Check if it's a timeout-related error
        String statusName = status.getName().toLowerCase();
        if (statusName.contains("timeout")
            || statusName.contains("notupdate")
            || statusName.contains("stale")) {
          health.timeoutCount++;
        } else {
          health.errorCount++;
        }
        health.lastErrorTime = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();
        failedSignals.add(String.format("%s: %s", signal.getName(), status.getName()));
      }
    }

    // Report any failures immediately
    if (!failedSignals.isEmpty()) {
      String errorMsg =
          String.format(
              "[%s] CAN Signal Failures: %s", subsystemName, String.join(", ", failedSignals));
      DriverStation.reportWarning(errorMsg, false);
      Logger.recordOutput("CANDiagnostics/" + subsystemName + "/FailedSignals", errorMsg);
    }
  }

  /**
   * Check if all signals in the array are healthy (OK status).
   *
   * @param signals Array of signals to check
   * @return true if all signals are OK, false otherwise
   */
  public static boolean areSignalsHealthy(BaseStatusSignal... signals) {
    if (signals == null || signals.length == 0) return true;

    for (BaseStatusSignal signal : signals) {
      StatusCode status = signal.getStatus();
      if (status != StatusCode.OK) {
        return false;
      }
    }
    return true;
  }

  /**
   * Periodically log health statistics for all tracked signals. Call this from
   * Robot.robotPeriodic() or similar.
   */
  public static void periodicHealthCheck() {
    double currentTime = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();

    if (currentTime - lastHealthCheckTime < HEALTH_CHECK_PERIOD) {
      return;
    }
    lastHealthCheckTime = currentTime;

    // Log overall health statistics
    Map<String, Integer> subsystemTimeouts = new HashMap<>();
    Map<String, Integer> subsystemErrors = new HashMap<>();

    for (Map.Entry<String, SignalHealth> entry : signalHealthMap.entrySet()) {
      String key = entry.getKey();
      SignalHealth health = entry.getValue();

      String subsystem = key.split("/")[0];

      if (health.timeoutCount > 0) {
        subsystemTimeouts.merge(subsystem, health.timeoutCount, Integer::sum);
      }
      if (health.errorCount > 0) {
        subsystemErrors.merge(subsystem, health.errorCount, Integer::sum);
      }

      // Log detailed signal health
      Logger.recordOutput("CANDiagnostics/" + key + "/Success", health.successCount);
      Logger.recordOutput("CANDiagnostics/" + key + "/Timeouts", health.timeoutCount);
      Logger.recordOutput("CANDiagnostics/" + key + "/Errors", health.errorCount);
      Logger.recordOutput("CANDiagnostics/" + key + "/LastStatus", health.lastStatus.getName());

      // Report persistent issues
      if (health.timeoutCount > 10 && currentTime - health.lastErrorTime < 1.0) {
        DriverStation.reportWarning(
            String.format(
                "CAN Signal '%s' has %d timeouts - check device connection!",
                key, health.timeoutCount),
            false);
      }
    }

    // Log subsystem-level summaries
    for (Map.Entry<String, Integer> entry : subsystemTimeouts.entrySet()) {
      Logger.recordOutput(
          "CANDiagnostics/Summary/" + entry.getKey() + "/TotalTimeouts", entry.getValue());
    }
    for (Map.Entry<String, Integer> entry : subsystemErrors.entrySet()) {
      Logger.recordOutput(
          "CANDiagnostics/Summary/" + entry.getKey() + "/TotalErrors", entry.getValue());
    }
  }

  /**
   * Get a human-readable report of CAN device health issues.
   *
   * @return String report of problematic devices
   */
  public static String getHealthReport() {
    StringBuilder report = new StringBuilder();
    report.append("CAN Device Health Report:\n");

    boolean hasIssues = false;
    for (Map.Entry<String, SignalHealth> entry : signalHealthMap.entrySet()) {
      SignalHealth health = entry.getValue();
      if (health.timeoutCount > 0 || health.errorCount > 0) {
        hasIssues = true;
        report.append(
            String.format(
                "  %s: %d timeouts, %d errors (status: %s)\n",
                entry.getKey(),
                health.timeoutCount,
                health.errorCount,
                health.lastStatus.getName()));
      }
    }

    if (!hasIssues) {
      report.append("  All devices healthy!\n");
    }

    return report.toString();
  }

  /** Reset all health statistics. Useful for testing or after fixing issues. */
  public static void resetHealthStats() {
    signalHealthMap.clear();
  }
}
