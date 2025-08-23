// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;

/**
 * This class defines the runtime mode used by AdvantageKit. The mode is always "real" when running
 * on a roboRIO. Change the value of "simMode" to switch between "sim" (physics sim) and "replay"
 * (log replay from a file).
 */
public final class Constants {
  public static final Mode simMode = Mode.SIM;
  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }
  // ====================Intake (3_)====================
  public static class IntakeConstants {

    public static final int intakePivotID = 30;
    public static final int intakeRollerID = 31;

    // Sensor IDs
    public static final int CANCODER_ID = 33;
    public static final int CANRANGE_ID = 34;

    // Gear ratios
    public static final double PIVOT_GEAR_RATIO = 61.71; // X44- (pivot slap down): (61.71 : 1)
    public static final double L1_BAR_GEAR_RATIO = 1.0 / 3.0; // X44- L1 bar: (1:3)
    public static final double ROLLER_GEAR_RATIO = 5.56; // X44- Rollers: (5.56 : 1)

    // Current limits
    public static final double MAX_CURRENT_LIMIT = 30.0; // Amps

    // PID constants
    public static final double KG = 0.0; // Gravity feedforward
    public static final double KP = 0.0; // Proportional gain
    public static final double KI = 0.0; // Integral gain
    public static final double KD = 0.0; // Derivative gain

    // Motion constraints
    public static final double MAX_ACCEL = 0.0; // rad/s^2
    public static final double MAX_VELOCITY = 0.0; // rad/s
  }

  // ====================Elevator (4_)====================
  public static class ElevatorConstants {

    public static final int elevatorRightPort = 40;
    public static final int elevatorLeftPort = 41;
    public static final int elevatorExtraPort = 42;
  }

  // ====================End Effector (5_)====================
  public static class EndEffectorConstants {
    public static final int endEffectorPivotPort = 50;
    public static final int endEffectorRollerPort = 51;
  }

  // ====================Climb (6_)====================
  public static class ClimbConstants {

    public static final double reduction = (23.11/1);


    public static final int ID = 60;
    public static final double GearRatio = 0.0;
    public static final double motorIDS = 0.0;
    public static final double maxcurrentLimit = 0.0;
    public static final double kG = 0.0;
    public static final double kP = 0.0;
    public static final double kI = 0.0;
    public static final double kD = 0.0;





  }

  // ====================Indexer (7_)====================
  public static class IndexerConstants {
    public static final int indexerRightPort = 70;
    public static final int indexerLeftPort = 71;
    public static final int indexerCANrange = 72;
  }

  public record PIDgains(
      double kP, double kI, double kD, double kS, double kV, double kA, double kG) {

    // Pure PID (the average use case)
    public PIDgains(double kP, double kI, double kD) {
      this(kP, kI, kD, 0, 0, 0, 0);
    }

    // Arm/Elevator PID (we don't care about voltage or acceleration)
    public PIDgains(double kP, double kI, double kD, double kS, double kG) {
      this(kP, kI, kD, kS, 0, 0, kG);
    }

    // Flywheels/Wheels (since kG is not an issue)
    public PIDgains(double kP, double kI, double kD, double kS, double kV, double kA) {
      this(kP, kI, kD, kS, kV, kA, 0);
    }
  }
}
