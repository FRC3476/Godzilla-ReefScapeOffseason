package frc.robot.util;

public class MotorStallDetection {
  public static boolean isMotorStalled(
      double motorCurrent, double motorVelocity, double currentLimitAmps, double velocityLimitRPS) {
    return motorCurrent > currentLimitAmps && motorVelocity < velocityLimitRPS;
  }
}
