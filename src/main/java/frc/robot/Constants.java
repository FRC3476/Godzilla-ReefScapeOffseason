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

    public static final int intakePivotPort = 30;
    public static final int intakeRollerPort = 31;
    public static final int intakeL1StopperPort = 32;
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
