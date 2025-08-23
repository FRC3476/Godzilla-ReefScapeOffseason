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

import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
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

    public static final double ELEVATOR_kP = 0;
    public static final double ELEVATOR_kI = 0;
    public static final double ELEVATOR_kD = 0;
    public static final double ELEVATOR_kG = 0;

    public static final double ELEVATOR_Velo = 0;
    public static final double ELEVATOR_Accel = 0;
    public static final double ELEVATOR_Jerk = 0;

    public static final double ELEVATOR_CURRENT_LIMIT_AMPS = 80;

    public static final double ELEVATOR_SETPOINT_TOLERANCE_INCH = 1;

    public static final TalonFXConfiguration elevatorRightTalon =
        new TalonFXConfiguration()
            .withSlot0(
                new Slot0Configs()
                    .withKP(ELEVATOR_kP)
                    .withKI(ELEVATOR_kI)
                    .withKD(ELEVATOR_kD)
                    .withKG(ELEVATOR_kG)
                    .withGravityType(GravityTypeValue.Elevator_Static))
            .withMotionMagic(
                new MotionMagicConfigs()
                    .withMotionMagicCruiseVelocity(ELEVATOR_Velo)
                    .withMotionMagicAcceleration(ELEVATOR_Accel)
                    .withMotionMagicJerk(ELEVATOR_Jerk))
            .withMotorOutput(
                new MotorOutputConfigs()
                    .withInverted(InvertedValue.Clockwise_Positive)
                    .withNeutralMode(NeutralModeValue.Brake))
            .withCurrentLimits(
                new CurrentLimitsConfigs()
                    .withStatorCurrentLimitEnable(true)
                    .withStatorCurrentLimit(ELEVATOR_CURRENT_LIMIT_AMPS));

    public static final TalonFXConfiguration elevatorLeftTalon =
        new TalonFXConfiguration()
            .withSlot0(
                new Slot0Configs()
                    .withKP(ELEVATOR_kP)
                    .withKI(ELEVATOR_kI)
                    .withKD(ELEVATOR_kD)
                    .withKG(ELEVATOR_kG)
                    .withGravityType(GravityTypeValue.Elevator_Static))
            .withMotionMagic(
                new MotionMagicConfigs()
                    .withMotionMagicCruiseVelocity(ELEVATOR_Velo)
                    .withMotionMagicAcceleration(ELEVATOR_Accel)
                    .withMotionMagicJerk(ELEVATOR_Jerk))
            .withMotorOutput(
                new MotorOutputConfigs()
                    .withInverted(InvertedValue.CounterClockwise_Positive)
                    .withNeutralMode(NeutralModeValue.Brake))
            .withCurrentLimits(
                new CurrentLimitsConfigs()
                    .withStatorCurrentLimitEnable(true)
                    .withStatorCurrentLimit(ELEVATOR_CURRENT_LIMIT_AMPS));

    public enum ElevatorState {
      DEFAULT,
      JOG_UP,
      JOG_DOWN,
      STOW_RESET,
      // ========
      COLLECT_CORAL_GROUND,
      COLLECT_CORAL_STATION,
      STAGE_CORAL_HANDOFF,
      RESET_CORAL_HANDOFF,
      COLLECT_BOTTOM_ALGAE,
      COLLECT_TOP_ALGAE,
      STOW_ALGAE,
      // ========
      SCORE_CORAL_L2,
      SCORE_CORAL_L3,
      SCORE_CORAL_L4,
      SCORE_ALGAE_PROCESSOR,
      SCORE_ALGAE_NET,
    }

    // ========Elevator Constant Positions========
    public static final double ELEVATOR_ZERO_SETPOINT_INCH = 0.0;
    public static final double ELEVATOR_INTAKE_SETPOINT_INCH = 22.0; // 22
    public static final double ELEVATOR_HANDOFF_RESET_SETPOINT_INCH = 21.0;
    public static final double ELEVATOR_SOURCE_SETPOINT_INCH = 26.0;
    public static final double ELEVATOR_ALGAE_STOW_SETPOINT_INCH = 10.0;
    public static final double ELEVATOR_HANDOFF_SETPOINT_INCH = 20.0; // 18.5
    public static final double ELEVATOR_BOTTOM_ALGAE_PULL_SETPOINT_INCH = 20.0;
    public static final double ELEVATOR_TOP_ALGAE_PULL_SETPOINT_INCH = 27.5;

    public static final double ELEVATOR_L2_SETPOINT_INCH = 7.5;
    public static final double ELEVATOR_L3_SETPOINT_INCH = 17.0; // 18.75
    public static final double ELEVATOR_L4_SETPOINT_INCH = 30.0;
    public static final double ELEVATOR_NET_SETPOINT_INCH = 30.0;
    public static final double ELEVATOR_PROCESSOR_SETPOINT_INCH = 4.0;

    public static final double ELEVATOR_JOG_UP_DUTY = 0.15;
    public static final double ELEVATOR_JOG_DOWN_DUTY = -0.15;
  }

  // ====================End Effector (5_)====================
  public static class EndEffectorConstants {
    public static final int endEffectorPivotPort = 50;
    public static final int endEffectorRollerPort = 51;
  }

  // ====================Climb (6_)====================
  public static class ClimbConstants {
    public static final int climbPort = 60;
  }

  // ====================Indexer (7_)====================
  public static class IndexerConstants {
    public static final int indexerRightPort = 70;
    public static final int indexerLeftPort = 71;
    public static final int indexerCANrange = 72;
  }

  // ====================Physical Constants====================
  public static class PhysicalConstants {
    public static final double ABSOLUTE_ZERO = 0.0;
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
