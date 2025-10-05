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

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import java.util.Arrays;

/**
 * This class defines the runtime mode used by AdvantageKit. The mode is always "real" when running
 * on a roboRIO. Change the value of "simMode" to switch between "sim" (physics sim) and "replay"
 * (log replay from a file).
 */
public final class Constants {
  public static final boolean tuningMode = true;
  public static boolean disableHAL = false;

  public static void disableHAL() {
    disableHAL = true;
  }

  public static final Mode simMode = Mode.SIM;
  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

  public static final double LOOP_PERIOD_SECS = 0.02;
  public static final long PIVOT_ZERO_SLEEP_MS = 200;

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

  public static String DRIVE_CANIVORE_NAME = "DRIVE";
  public static String MISC_CANIVORE_NAME = "MISC";
  public static CANBus DRIVE_CANIVORE = new CANBus(DRIVE_CANIVORE_NAME);
  public static CANBus MISC_CANIVORE = new CANBus(MISC_CANIVORE_NAME);

  // ====================Drive (0_ and 1_)====================
  public static class DriveConstants {
    // Acceleration limits
    // Large numnbers so they don't do anything.
    public static final double MAX_TRANSLATIONAL_ACCEL = 3476.0; // m/s²
    public static final double MAX_ROTATIONAL_ACCEL = 3476.0; // rad/s²

    // Dynamic acceleration limit formula weights for: E - elevator.height*b -
    // intakePivot.height*c-(endeffectorpivot.height*a+elevator.height)*d
    public static final double DYNAMIC_ACCEL_WEIGHT_A = 0.0; // Weight for endEffectorPivotPosition
    public static final double DYNAMIC_ACCEL_WEIGHT_B = 0.0; // Weight for elevator height
    public static final double DYNAMIC_ACCEL_WEIGHT_C =
        0.0; // Weight for intake pivot height reduction
    public static final double DYNAMIC_ACCEL_WEIGHT_D =
        0.0; // Weight for combined end effector and elevator height reduction

    // Slip Current Characterization Constants (Wall Test)
    public static final double SLIP_START_DELAY = 0.0; // Secs
    public static final double SLIP_RAMP_RATE = -0.5; // Volts/Sec
    public static final double SLIP_MAX_VOLTAGE = 3476.0; // Volts
    public static final double SLIP_VELOCITY_THRESHOLD =
        3476.0; // Velocity derivative indicating wheels started spinning
    public static final double SLIP_MIN_CURRENT_THRESHOLD = 0.0; // Minimum current threshold
  }

  // ====================Intake (3_)====================
  public static class IntakeConstants {

    // Motor IDs
    public static final int intakePivotID = 30;
    public static final int intakeRollerID = 31;
    public static final int intakelvl1BlockerID = 32;
    // Sensor IDs
    public static final int CANCODER_ID = 33;
    public static final int CANRANGE_ID = 34;

    // Pivot position for L1 scoring (radians)
    public static final double SCORE_PREPPED_L1_PIVOT_POSITION_ROTATIONS =
        Units.degreesToRotations(65.7874127);
    // Stowed position for intake pivot
    public static final double INTAKE_PIVOT_STOWED_POSITION = 0.0;

    public static final double SCORE_PREPPED_L1_ROLLER_VOLTS = 0.0;

    // Setpoints
    public static final double PIVOT_TOLERANCE_ROTATIONS = 0.0;
    public static final double PIVOT_L1_SETPOINT_ROTATIONS = Units.degreesToRotations(65.7874127);
    public static final double ROLLER_L1_SETPOINT_VOLTS = 0.0;

    // Pivot Positions
    public static final double PIVOT_INTAKE_POSITION =
        Units.degreesToRotations(-40); // Intake down angle
    public static final double PIVOT_UP_POSITION =
        Units.degreesToRotations(104.5837512); // Intake up angle
    public static final double PIVOT_SCORING_POSITION =
        Units.degreesToRotations(65.7874127); // L1 scoring position
    public static final double SCORING_PREP_PIVOT_POSITION_ROTATIONS = 0.0;

    // L1 Blocker Positions
    public static final double L1_BLOCKER_ENGAGED_POSITION = 0.0;
    public static final double L1_BLOCKER_DISENGAGED_POSITION = 0.0;

    // Roller Voltages
    public static final double ROLLER_SCORING_OUT_VOLTS = 0.0;

    public enum IntakeState {
      NONE,
      STOW,
      INTAKE_L1,
      INTAKE,
      REJECT_CORAL,
      IDLE,
      HAND_OFF,
      SCORING,
      SCORING_PREP,
      JAM_DETECTED
    }

    // Gear ratios
    public static final double PIVOT_GEAR_RATIO = 1.0 / 61.71;
    public static final double PIVOT_RTS = 16; // X44- (pivot slap down): (61.71 : 1)
    public static final double PIVOT_STM = 54 / 12;

    public static final double L1_BAR_GEAR_RATIO = 1.0 / 3.0; // X44- L1 bar: (1:3)
    public static final double ROLLER_GEAR_RATIO = 1.0 / 5.56; // X44- Rollers: (5.56 : 1)

    // MOI
    public static final double PIVOT_MOI = 0.01;
    public static final double L1_BAR_MOI = 0.01;
    public static final double ROLLER_MOI = 0.001;

    // Current limits
    // Roller, L1, Pivot
    // public static final double MAX_SUPPLY_CURRENT_LIMIT = 50.0; // Amps
    public static final double ROLLER_MAX_SUPPLY_CURRENT_LIMIT = 40.0; // Amps
    public static final double L1_MAX_SUPPLY_CURRENT_LIMIT = 40.0; // Amps
    public static final double PIVOT_MAX_SUPPLY_CURRENT_LIMIT = 40.0; // Amps

    public static final double ROLLER_MAX_STATOR_CURRENT_LIMIT = 40.0; // Amps
    public static final double L1_MAX_STATOR_CURRENT_LIMIT = 40.0; // Amps
    public static final double PIVOT_MAX_STATOR_CURRENT_LIMIT = 40.0; // Amps

    // PID constants
    public static final double Tuneable_pivotKP = 50; // Proportional gain
    public static final double Tuneable_pivotKI = 0.0; // Integral gain
    public static final double Tuneable_pivotKD = 0.0; // Derivative gain
    public static final double Tuneable_pivotKG = 0.425; // Gravity feedforward
    public static final double Tuneable_pivotKS = 0.125; // Gravity feedforward

    // Motion constraints
    public static final double Tuneable_pivot_ACCEL = 1000; // rad/s^2
    public static final double Tuneable_pivot_VELOCITY = 50; // rad/s
    public static final double Tuneable_pivotJERK = 1000; // rad/s^3

    // PID constants
    public static final double lvl1blockerKG = 0.0; // Gravity feedforward
    public static final double lvl1blockerKP = 0.0; // Proportional gain
    public static final double lvl1blockerKI = 0.0; // Integral gain
    public static final double lvl1blockerKD = 0.0; // Derivative gain

    // Motion constraints
    public static final double lvl1blockerMAX_ACCEL = 0.0; // rad/s^2
    public static final double lvl1blockerMAX_VELOCITY = 0.0; // rad/s
    public static final double lvl1blockerJERK = 0.0; // rad/s^3

    public static final double L1_BLOCKER_CORAL_ENGAGED_POSITION = 0.0; // radians
    public static final double L1_BLOCKER_CORAL_DISENGAGED_POSITION = 0.0; // radians

    // Stall detection
    public static final double ROLLER_STALLED_CURRENT_A = 1000;
    public static final double ROLLER_STALLED_RPS = 0;
    public static final double DEJAM_DEBOUNCE_SECONDS = 0.1;

    public static final TalonFXConfiguration PIVOT_TALON_CONFIG =
        new TalonFXConfiguration()
            .withSlot0(
                new Slot0Configs()
                    .withKP(Tuneable_pivotKP)
                    .withKI(Tuneable_pivotKI)
                    .withKD(Tuneable_pivotKD)
                    .withKG(Tuneable_pivotKG)
                    .withGravityType(GravityTypeValue.Arm_Cosine))
            .withMotionMagic(
                new MotionMagicConfigs()
                    .withMotionMagicCruiseVelocity(Tuneable_pivot_VELOCITY)
                    .withMotionMagicAcceleration(Tuneable_pivot_ACCEL)
                    .withMotionMagicJerk(Tuneable_pivotJERK))
            .withMotorOutput(
                new MotorOutputConfigs()
                    .withInverted(InvertedValue.Clockwise_Positive)
                    .withNeutralMode(NeutralModeValue.Brake))
            .withFeedback(
                new FeedbackConfigs()
                    .withRotorToSensorRatio(PIVOT_RTS)
                    .withFeedbackRemoteSensorID(CANCODER_ID)
                    .withSensorToMechanismRatio(PIVOT_STM)
                    .withFeedbackSensorSource(FeedbackSensorSourceValue.RemoteCANcoder))
            .withCurrentLimits(
                new CurrentLimitsConfigs()
                    .withSupplyCurrentLimitEnable(true)
                    .withSupplyCurrentLimit(PIVOT_MAX_SUPPLY_CURRENT_LIMIT));

    public static final CANcoderConfiguration CANCODER_CONFIG =
        new CANcoderConfiguration()
            .withMagnetSensor(
                new MagnetSensorConfigs()
                    .withSensorDirection(SensorDirectionValue.CounterClockwise_Positive)
                    .withAbsoluteSensorDiscontinuityPoint(0.5));

    public static final TalonFXConfiguration L1Bar_TALON_CONFIG =
        new TalonFXConfiguration()
            .withSlot0(
                new Slot0Configs()
                    .withKP(lvl1blockerKP)
                    .withKI(lvl1blockerKI)
                    .withKD(lvl1blockerKD)
                    .withKG(lvl1blockerKG)
                    .withGravityType(GravityTypeValue.Arm_Cosine))
            .withMotionMagic(
                new MotionMagicConfigs()
                    .withMotionMagicCruiseVelocity(lvl1blockerMAX_VELOCITY)
                    .withMotionMagicAcceleration(lvl1blockerMAX_ACCEL)
                    .withMotionMagicJerk(lvl1blockerJERK))
            .withMotorOutput(
                new MotorOutputConfigs()
                    .withInverted(InvertedValue.CounterClockwise_Positive)
                    .withNeutralMode(NeutralModeValue.Brake))
            .withCurrentLimits(
                new CurrentLimitsConfigs()
                    .withSupplyCurrentLimitEnable(true)
                    .withSupplyCurrentLimit(L1_MAX_SUPPLY_CURRENT_LIMIT));

    public static final TalonFXConfiguration ROLLER_TALON_CONFIG =
        new TalonFXConfiguration()
            .withMotorOutput(
                new MotorOutputConfigs()
                    .withInverted(InvertedValue.CounterClockwise_Positive)
                    .withNeutralMode(NeutralModeValue.Brake))
            .withCurrentLimits(
                new CurrentLimitsConfigs()
                    .withSupplyCurrentLimitEnable(true)
                    .withSupplyCurrentLimit(PIVOT_MAX_SUPPLY_CURRENT_LIMIT));
  }

  // ====================Elevator (4_)====================
  public static class ElevatorConstants {
    public static final int elevatorRightID = 40;
    public static final int elevatorLeftID = 41;
    public static final int elevatorExtraID = 42;

    public static final double Tunable_ELEVATOR_kP = 2;
    public static final double Tunable_ELEVATOR_kI = 0;
    public static final double Tunable_ELEVATOR_kD = 0.1;
    public static final double Tunable_ELEVATOR_kG = 0.04;
    public static final double Tunable_ELEVATOR_kS = 0.2;

    public static final double Tunable_ELEVATOR_Velo = 300;
    public static final double Tunable_ELEVATOR_Accel = 3000;
    public static final double Tunable_ELEVATOR_Jerk = 10000;

    public static final double ELEVATOR_CURRENT_LIMIT_AMPS = 80;

    public static final double ELEVATOR_SETPOINT_TOLERANCE_INCH = 1;
    public static final double ELEVATOR_MOTOR_TO_SENSOR_RATIO =
        1 / (1.8427325868896291219526481028964);

    public static final double STALLED_CURRENT = 1000;
    public static final double STALLED_RPS = 0.0;
    public static final double STALLED_TOLERANCE_INCHES = 2.0;
    public static final double DEJAM_DISTANCE_INCHES = 12.0;
    public static final double DEJAM_DEBOUNCE_SECONDS = 0.1;

    public static final TalonFXConfiguration elevatorRightTalon =
        new TalonFXConfiguration()
            .withSlot0(
                new Slot0Configs()
                    .withKP(Tunable_ELEVATOR_kP)
                    .withKI(Tunable_ELEVATOR_kI)
                    .withKD(Tunable_ELEVATOR_kD)
                    .withKG(Tunable_ELEVATOR_kG)
                    .withGravityType(GravityTypeValue.Elevator_Static))
            .withFeedback(
                new FeedbackConfigs()
                    .withFeedbackSensorSource(FeedbackSensorSourceValue.RotorSensor)
                    .withSensorToMechanismRatio(ELEVATOR_MOTOR_TO_SENSOR_RATIO))
            .withMotionMagic(
                new MotionMagicConfigs()
                    .withMotionMagicCruiseVelocity(Tunable_ELEVATOR_Velo)
                    .withMotionMagicAcceleration(Tunable_ELEVATOR_Accel)
                    .withMotionMagicJerk(Tunable_ELEVATOR_Jerk))
            .withMotorOutput(
                new MotorOutputConfigs()
                    .withInverted(InvertedValue.Clockwise_Positive)
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
    public static final double ELEVATOR_MAX_SETPOINT_INCH = 53.4375; // max height
    public static final double MAX_HEIGHT_INCH = 53.4375;

    public static final double ELEVATOR_L2_SETPOINT_INCH = 7.078988;
    public static final double ELEVATOR_L3_SETPOINT_INCH = 23.003301;
    public static final double ELEVATOR_L4_SETPOINT_INCH = 53.4375;
    public static final double ELEVATOR_NET_SETPOINT_INCH = 30.0;
    public static final double ELEVATOR_PROCESSOR_SETPOINT_INCH = 0.0;

    public static final double ELEVATOR_JOG_UP_DUTY = 0.15;
    public static final double ELEVATOR_JOG_DOWN_DUTY = -0.15;

    // Homing sequence constants
    public static final double ELEVATOR_HOMING_VOLTAGE = -0.0; // Downward voltage for homing
    public static final double HOMING_TIMEOUT_SECONDS = 3476.0; // Max time to allow for homing

    // Coral scoring heights
    public static final double ELEVATOR_L2_AGAINST_REEF_SETPOINT_INCH = 14.418111;
    public static final double ELEVATOR_L3_AGAINST_REEF_SETPOINT_INCH = 30.029785;
    public static final double ELEVATOR_L4_AGAINST_REEF_SETPOINT_INCH = 53.4375;

    // Algae scoring heights
    public static final double ELEVATOR_L2_ALGAE_AGAINST_REEF_SETPOINT_INCH = 30.907161;
    public static final double ELEVATOR_L3_ALGAE_AGAINST_REEF_SETPOINT_INCH = 45.325558;

    // Barge heights
    public static final double ELEVATOR_BARGE_BACK_SETPOINT_INCH = 53.4375;
    public static final double ELEVATOR_BARGE_FRONT_SETPOINT_INCH = 53.4375;

    public static final double kElevatorDrumRadius = Units.inchesToMeters(1.128);
    public static final double kGearing = (13.0 / 50.0);
    public static final double kElevatorUnitToRotorRatio =
        kGearing * 2.0 * kElevatorDrumRadius * Math.PI;

    public static final double GEAR_RATIO =
        ElevatorConstants.kElevatorUnitToRotorRatio; // Adjust based on your gearing
    public static final double CARRIAGE_MASS_KG = 1.97312681; // Mass of elevator carriage
    public static final double DRUM_RADIUS_METERS =
        ElevatorConstants.kElevatorDrumRadius; // Radius of drum/pulley
    public static final double MIN_HEIGHT_METERS = 0.0; // Minimum elevator height
    public static final double MAX_HEIGHT_METERS = 1.0; // Maximum elevator height
  }

  // ====================End Effector (5_)====================
  public static class EndEffectorConstants {
    public static final int pivotID = 50;
    public static final int rollerID = 51;
    public static final int FIRST_CORAL_CANRANGE_ID = 52;
    public static final int SECOND_CORAL_CANRANGE_ID = 53;
    public static final int PIVOT_CANCODER_ID = 54;

    public static final double Tunable_PIVOT_kP = 35;
    public static final double Tunable_PIVOT_kI = 0;
    public static final double Tunable_PIVOT_kD = 0.5;
    public static final double Tunable_PIVOT_kG = 0.615;
    public static final double Tunable_PIVOT_kS = 0.135;

    public static final double Tunable_PIVOT_Velo = 1000;
    public static final double Tunable_PIVOT_Accel = 12;
    public static final double Tunable_PIVOT_Jerk = 1000;

    public static final double PIVOT_CURRENT_LIMIT_AMPS = 40;

    public static final double ROLLER_kP = 0;
    public static final double ROLLER_kI = 0;
    public static final double ROLLER_kD = 0;
    public static final double ROLLER_kS = 0;
    public static final double ROLLER_kA = 0;

    public static final double ROLLER_CURRENT_LIMIT_AMPS = 80;

    public static final double ALGAE_GEAR_RATIO = 1.0 / 12.22;
    public static final double CORAL_GEAR_RATIO = 1.0 / 6.11;

    public static final double PIVOT_RTS = 10;
    public static final double PIVOT_STM = 4;
    public static final double PIVOT_GEAR_RATIO = PIVOT_RTS * PIVOT_STM;

    public static final double ROLLER_STALLED_CURRENT = 1000.0;
    public static final double ROLLER_STALLED_RPS = 0.0;

    public static final double CLAW_HOLD_ALGAE_AMPS = 60.0;
    public static final double PIVOT_TOLERANCE_ROTATIONS = (double) 5 / 360;

    // ========End Effector Constant Positions========
    // Pivot positions in rotations
    public static final double IDLE_ANGLE_ROTATIONS = EndEffectorConstants.MIN_ANGLE_ROTATIONS;
    public static final double GROUND_ALGAE_ANGLE_ROTATIONS = Units.degreesToRotations(-52.0336836);
    public static final double ALGAE_IDLE_ANGLE_ROTATIONS = Units.degreesToRotations(-38.3080987);
    public static final double PROCESSOR_ANGLE_ROTATIONS = Units.degreesToRotations(-38.3080987);
    public static final double L2_L3_AGAINST_REEF_ANGLE_ROTATIONS =
        Units.degreesToRotations(-16.3769186);
    public static final double L2_L3_AWAY_FROM_REEF_ANGLE_ROTATIONS =
        Units.degreesToRotations(17.7998883);
    public static final double L4_AWAY_FROM_REEF_ANGLE_ROTATIONS =
        Units.degreesToRotations(3.1972053);
    public static final double ALGAE_REMOVAL_ANGLE_ROTATIONS =
        Units.degreesToRotations(-56.8542103);
    public static final double BARGE_FORWARD_ANGLE_ROTATIONS = Units.degreesToRotations(43.8547133);
    public static final double BARGE_BACKWARD_ANGLE_ROTATIONS =
        Units.degreesToRotations(119.8473749);
    public static final double PIVOT_ABSOLUTE_ENCODER_OFFSET = 0.305908;

    // Hardstop angles
    public static final double UPPER_HARDSTOP_ANGLE_ROTATIONS =
        Units.degreesToRotations(119.8473749);
    public static final double LOWER_HARDSTOP_ANGLE_ROTATIONS =
        EndEffectorConstants.MIN_ANGLE_ROTATIONS;
    // Standardized angle constants with RADIAN suffix
    public static final double MAX_ANGLE_ROTATIONS = Units.degreesToRotations(119.8473749);
    public static final double MIN_ANGLE_ROTATIONS = Units.degreesToRotations(-92.16);
    public static final double MAX_SAFE_ANGLE_ROTATIONS =
        Units.degreesToRotations(35); // old value 53.9126895
    public static final double MIN_SAFE_ANGLE_ROTATIONS =
        Units.degreesToRotations(-21); // old value -61.1115004

    public static final TalonFXConfiguration PIVOT_TALON_CONFIG =
        new TalonFXConfiguration()
            .withSlot0(
                new Slot0Configs()
                    .withKP(Tunable_PIVOT_kP)
                    .withKI(Tunable_PIVOT_kI)
                    .withKD(Tunable_PIVOT_kD)
                    .withKG(Tunable_PIVOT_kG)
                    .withGravityType(GravityTypeValue.Arm_Cosine))
            .withMotionMagic(
                new MotionMagicConfigs()
                    .withMotionMagicCruiseVelocity(Tunable_PIVOT_Velo)
                    .withMotionMagicAcceleration(Tunable_PIVOT_Accel)
                    .withMotionMagicJerk(Tunable_PIVOT_Jerk))
            .withMotorOutput(
                new MotorOutputConfigs()
                    .withInverted(InvertedValue.Clockwise_Positive)
                    .withNeutralMode(NeutralModeValue.Brake))
            .withFeedback(
                new FeedbackConfigs()
                    .withRotorToSensorRatio(PIVOT_RTS)
                    .withFeedbackRemoteSensorID(PIVOT_CANCODER_ID)
                    .withSensorToMechanismRatio(PIVOT_STM)
                    .withFeedbackSensorSource(FeedbackSensorSourceValue.RemoteCANcoder))
            .withCurrentLimits(
                new CurrentLimitsConfigs()
                    .withStatorCurrentLimitEnable(true)
                    .withStatorCurrentLimit(PIVOT_CURRENT_LIMIT_AMPS));

    public static final TalonFXConfiguration PIVOT_TALON_MOVING_CONFIG =
        new TalonFXConfiguration()
            .withSlot0(
                new Slot0Configs()
                    .withKP(Tunable_PIVOT_kP)
                    .withKI(Tunable_PIVOT_kI)
                    .withKD(Tunable_PIVOT_kD)
                    .withKG(Tunable_PIVOT_kG)
                    .withGravityType(GravityTypeValue.Arm_Cosine))
            .withMotionMagic(
                new MotionMagicConfigs()
                    .withMotionMagicCruiseVelocity(Tunable_PIVOT_Velo)
                    .withMotionMagicAcceleration(Tunable_PIVOT_Accel)
                    .withMotionMagicJerk(Tunable_PIVOT_Jerk))
            .withMotorOutput(
                new MotorOutputConfigs()
                    .withInverted(InvertedValue.Clockwise_Positive)
                    .withNeutralMode(NeutralModeValue.Brake))
            .withFeedback(
                new FeedbackConfigs()
                    .withRotorToSensorRatio(PIVOT_RTS)
                    .withFeedbackRemoteSensorID(PIVOT_CANCODER_ID)
                    .withSensorToMechanismRatio(PIVOT_STM)
                    .withFeedbackSensorSource(FeedbackSensorSourceValue.RemoteCANcoder))
            .withCurrentLimits(
                new CurrentLimitsConfigs()
                    .withStatorCurrentLimitEnable(true)
                    .withStatorCurrentLimit(PIVOT_CURRENT_LIMIT_AMPS));

    public static final CANcoderConfiguration PIVOT_CANCODER_CONFIG =
        new CANcoderConfiguration()
            .withMagnetSensor(
                new MagnetSensorConfigs()
                    .withSensorDirection(SensorDirectionValue.CounterClockwise_Positive)
                    .withAbsoluteSensorDiscontinuityPoint(0.5));

    public static final TalonFXConfiguration ROLLER_TALON_CONFIG =
        new TalonFXConfiguration()
            .withSlot0(
                new Slot0Configs()
                    .withKP(ROLLER_kP)
                    .withKI(ROLLER_kI)
                    .withKD(ROLLER_kD)
                    .withKS(ROLLER_kS)
                    .withKA(ROLLER_kA))
            .withMotorOutput(
                new MotorOutputConfigs()
                    .withInverted(InvertedValue.Clockwise_Positive)
                    .withNeutralMode(NeutralModeValue.Brake))
            .withCurrentLimits(
                new CurrentLimitsConfigs()
                    .withStatorCurrentLimitEnable(true)
                    .withStatorCurrentLimit(ROLLER_CURRENT_LIMIT_AMPS));
  }

  // ====================Climb (6_)====================
  public static class ClimbConstants {

    public static final double reduction = (23.11 / 1);

    public static final int ID = 60;

    public static final double CLIMB_DEPLOY_POSITION = 0.0;
    public static final double CLIMB_CLIMB_POSITION = 0.0;
    public static final double STALL_AMPS = 1000.0;
    public static final double STALL_VELOCITY = 0.0;

    public static final double PIVOT_kP = 0;
    public static final double PIVOT_kI = 0;
    public static final double PIVOT_kD = 0;
    public static final double PIVOT_kG = 0;

    public static final double PIVOT_Velo = 0;
    public static final double PIVOT_Accel = 0;
    public static final double PIVOT_Jerk = 0;

    public static final double PIVOT_CURRENT_LIMIT_AMPS = 120;

    public static final TalonFXConfiguration CLIMB_TALON_CONFIG =
        new TalonFXConfiguration()
            .withMotorOutput(
                new MotorOutputConfigs()
                    .withInverted(InvertedValue.Clockwise_Positive)
                    .withNeutralMode(NeutralModeValue.Brake))
            .withCurrentLimits(
                new CurrentLimitsConfigs()
                    .withSupplyCurrentLimitEnable(true)
                    .withSupplyCurrentLimit(PIVOT_CURRENT_LIMIT_AMPS));
  }

  // ====================Feeder (2_)====================
  public static class FeederConstants {
    public static final int RIGHT_ID = 20;
    public static final int LEFT_ID = 21;
    public static final int CANRANGE_ID = 22;

    public static final double ROLLER_kP = 0;
    public static final double ROLLER_kI = 0;
    public static final double ROLLER_kD = 0;
    public static final double ROLLER_kS = 0;
    public static final double ROLLER_kA = 0;

    public static final double ROLLER_CURRENT_LIMIT_AMPS = 40;
    public static final double STALLED_CURRENT = 1000.0;
    public static final double STALLED_RPS = 0.0;
    public static final double DEJAM_DURATION_SECONDS = 0.05;
    public static final double DEJAM_DEBOUNCE_SECONDS = 0.1;

    public static final TalonFXConfiguration ROLLER_TALON_CONFIG =
        new TalonFXConfiguration()
            .withSlot0(
                new Slot0Configs()
                    .withKP(ROLLER_kP)
                    .withKI(ROLLER_kI)
                    .withKD(ROLLER_kD)
                    .withKS(ROLLER_kS)
                    .withKA(ROLLER_kA))
            .withMotorOutput(
                new MotorOutputConfigs()
                    .withInverted(InvertedValue.CounterClockwise_Positive)
                    .withNeutralMode(NeutralModeValue.Brake))
            .withCurrentLimits(
                new CurrentLimitsConfigs()
                    .withStatorCurrentLimitEnable(true)
                    .withStatorCurrentLimit(ROLLER_CURRENT_LIMIT_AMPS));

    public static final CANrangeConfiguration CANRANGE_CONFIG =
        new CANrangeConfiguration()
            .withProximityParams(
                new ProximityParamsConfigs()
                    .withProximityThreshold(0.05)
                    .withProximityHysteresis(0.01));

    public static final double FEEDER_IN_VOLTS = 12.0;
    public static final double FEEDER_OUT_VOLTS = -12.0;
    public static final double FEEDER_STOP_VOLTS = 0.0;
  }

  // ====================LED (8_)====================
  public static final class LEDConstants {
    public static final int ID = 19; // 80 not allowed, max ID is 62
    public static final int kNonCandleLEDCount = 10;
    public static final int kCandleLEDCount = 8;
    public static final int kMaxLEDCount = kNonCandleLEDCount + kCandleLEDCount;
    public static final double kLowBatteryThresholdVolts = 12.3;
  }

  // ====================Physical Constants====================
  public static class PhysicalConstants {
    public static final double ABSOLUTE_ZERO = 0.0;
  }

  public static class VisionConstants {
    public static final String DETECTION_LIMELIGHT = "limelight-center";
    public static final AprilTagFieldLayout fieldLayout =
        AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);
    public static final AprilTagFieldLayout kAprilTagLayout =
        AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);

    // Camera A (left side)
    public static final double kCameraAPitchDegrees = 15.0;
    public static final double kCameraAPitchRads = Units.degreesToRadians(kCameraAPitchDegrees);
    public static final double kCameraAHeightOffGroundMeters = Units.inchesToMeters(8.580998);
    public static final String kLimelightATableName = "limelight-left";
    public static final double kRobotToCameraAForward = Units.inchesToMeters(-10.365637);
    public static final double kRobotToCameraASide = Units.inchesToMeters(-11.422523);
    public static final Rotation2d kCameraAYawOffset = Rotation2d.fromDegrees(-151.13);

    // Camera B (right side)
    public static final double kCameraBPitchDegrees = 15.0;
    public static final double kCameraBPitchRads = Units.degreesToRadians(kCameraBPitchDegrees);
    public static final double kCameraBHeightOffGroundMeters = Units.inchesToMeters(8.580998);
    public static final String kLimelightBTableName = "limelight-right";
    public static final double kRobotToCameraBForward = Units.inchesToMeters(-11.422523);
    public static final double kRobotToCameraBSide = Units.inchesToMeters(10.365637);
    public static final Rotation2d kCameraBYawOffset = Rotation2d.fromDegrees(151.13);

    // Validation Constants
    public static final int kExpectedStdDevArrayLength = 12;

    // April Tags

    public static final int[] kAllowedTagIDs = {17, 18, 19, 20, 21, 22, 6, 7, 8, 9, 10, 11};

    public static final AprilTagFieldLayout kAprilTagLayoutReefsOnly =
        new AprilTagFieldLayout(
            kAprilTagLayout.getTags().stream()
                .filter(tag -> Arrays.stream(kAllowedTagIDs).anyMatch(element -> element == tag.ID))
                .toList(),
            kAprilTagLayout.getFieldLength(),
            kAprilTagLayout.getFieldWidth());
  }

  public static class SuperstructureConstants {
    public static double STOW_ELEVATOR_HEIGHT_INCH = 0.0;
    public static double STOW_ENDEFFECTOR_ROTATION_ROTATIONS =
        EndEffectorConstants.MIN_ANGLE_ROTATIONS;
    public static double STOW_INTAKE_ROTATION_ROTATIONS = Units.degreesToRotations(-26.9162484);

    public static double STOW_CORAL_ELEVATOR_HEIGHT_INCH = 0.0;
    public static double STOW_CORAL_ENDEFFECTOR_ROTATION_ROTATIONS =
        EndEffectorConstants.MIN_ANGLE_ROTATIONS;
    public static double STOW_CORAL_INTAKE_ROTATION_ROTATIONS =
        Units.degreesToRotations(-26.9162484);

    public static double STOW_ALGAE_ELEVATOR_HEIGHT_INCH = 0.0;
    public static double STOW_ALGAE_ENDEFFECTOR_ROTATION_ROTATIONS =
        Units.degreesToRotations(-38.3080987);
    public static double STOW_ALGAE_INTAKE_ROTATION_ROTATIONS =
        Units.degreesToRotations(-26.9162484);

    public static double INTAKE_CORAL_ELEVATOR_HEIGHT_INCH = 0.0;
    public static double INTAKE_CORAL_ENDEFFECTOR_ROTATION_ROTATIONS =
        EndEffectorConstants.MIN_ANGLE_ROTATIONS;
    public static double INTAKE_CORAL_INTAKE_ROTATION_ROTATIONS =
        Units.degreesToRotations(-26.9162484);

    public static double INTAKE_CORAL_L1_ELEVATOR_HEIGHT_INCH = 0.0;
    public static double INTAKE_CORAL_L1_ENDEFFECTOR_ROTATION_ROTATIONS =
        EndEffectorConstants.MIN_ANGLE_ROTATIONS;
    public static double INTAKE_CORAL_L1_INTAKE_ROTATION_ROTATIONS =
        Units.degreesToRotations(-26.9162484);

    public static double FEED_ELEVATOR_HEIGHT_INCH = 0.0;
    public static double FEED_ENDEFFECTOR_ROTATION_ROTATIONS =
        EndEffectorConstants.MIN_ANGLE_ROTATIONS;
    public static double FEED_INTAKE_ROTATION_ROTATIONS = Units.degreesToRotations(-26.9162484);

    public static double L1_PIVOT_ELEVATOR_HEIGHT_INCH = 0.0;
    public static double L1_PIVOT_ENDEFFECTOR_ROTATION_ROTATIONS =
        EndEffectorConstants.MIN_ANGLE_ROTATIONS;
    public static double L1_PIVOT_INTAKE_ROTATION_ROTATIONS = Units.degreesToRotations(65.7874127);

    public static double L2_AIM_ELEVATOR_HEIGHT_INCH = 14.41811 + 2;
    // Away from reef 7.078988deg
    public static double L2_AIM_ENDEFFECTOR_ROTATION_ROTATIONS =
        Units.degreesToRotations(-16.3769186);
    // Away from reef 17.7998883deg
    public static double L2_AIM_INTAKE_ROTATION_ROTATIONS = Units.degreesToRotations(-26.9162484);

    public static double L3_AIM_ELEVATOR_HEIGHT_INCH = 30.029785 + 2.5;
    // Away from reef 23.003301deg
    public static double L3_AIM_ENDEFFECTOR_ROTATION_ROTATIONS =
        Units.degreesToRotations(-16.3769186);
    // Away from reef 17.7998883deg
    public static double L3_AIM_INTAKE_ROTATION_ROTATIONS = Units.degreesToRotations(-26.9162484);

    public static double L4_AIM_ELEVATOR_HEIGHT_INCH = 53.4375;
    // Away from reef 53.4375
    public static double L4_AIM_ENDEFFECTOR_ROTATION_ROTATIONS =
        Units.degreesToRotations(3.1972053); // Away from reef (no against)
    public static double L4_AIM_INTAKE_ROTATION_ROTATIONS = Units.degreesToRotations(-26.9162484);

    public static double L1_SCORE_ELEVATOR_HEIGHT_INCH = 0.0;
    public static double L1_SCORE_ENDEFFECTOR_ROTATION_ROTATIONS =
        EndEffectorConstants.MIN_ANGLE_ROTATIONS;
    public static double L1_SCORE_INTAKE_ROTATION_ROTATIONS = Units.degreesToRotations(65.7874127);

    public static double L2_SCORE_ELEVATOR_HEIGHT_INCH = 14.41811 + 2;
    // Away from reef 7.078988deg
    public static double L2_SCORE_ENDEFFECTOR_ROTATION_ROTATIONS =
        Units.degreesToRotations(-16.3769186);
    // Away from reef 17.7998883deg
    public static double L2_SCORE_INTAKE_ROTATION_ROTATIONS = Units.degreesToRotations(-26.9162484);

    public static double L3_SCORE_ELEVATOR_HEIGHT_INCH = 30.029785 + 2.5;
    // Away from reef 23.003301deg
    public static double L3_SCORE_ENDEFFECTOR_ROTATION_ROTATIONS =
        Units.degreesToRotations(-16.3769186);
    // Away from reef 17.7998883deg
    public static double L3_SCORE_INTAKE_ROTATION_ROTATIONS = Units.degreesToRotations(-26.9162484);

    public static double L4_SCORE_ELEVATOR_HEIGHT_INCH = 53.4375;
    // Away from reef 53.4375
    public static double L4_SCORE_ENDEFFECTOR_ROTATION_ROTATIONS =
        Units.degreesToRotations(3.1972053); // Away from reef (no against)
    public static double L4_SCORE_INTAKE_ROTATION_ROTATIONS = Units.degreesToRotations(-26.9162484);

    public static double L2_FADEAWAY_ELEVATOR_HEIGHT_INCH = 14.41811;
    public static double L2_FADEAWAY_ENDEFFECTOR_ROTATION_ROTATIONS =
        Units.degreesToRotations(53.9126895);

    public static double L3_FADEAWAY_ELEVATOR_HEIGHT_INCH = 14.41811;
    public static double L3_FADEAWAY_ENDEFFECTOR_ROTATION_ROTATIONS =
        Units.degreesToRotations(53.9126895);

    public static double L4_FADEAWAY_ELEVATOR_HEIGHT_INCH = 53.4375;
    public static double L4_FADEAWAY_ENDEFFECTOR_ROTATION_ROTATIONS =
        Units.degreesToRotations(53.9126895);

    public static double ALGAE_HIGH_INTAKE_ELEVATOR_HEIGHT_INCH = 45.325558;
    public static double ALGAE_HIGH_INTAKE_ENDEFFECTOR_ROTATION_ROTATIONS =
        Units.degreesToRotations(-56.8542103);
    public static double ALGAE_HIGH_INTAKE_INTAKE_ROTATION_ROTATIONS =
        Units.degreesToRotations(-26.9162484);

    public static double ALGAE_LOW_INTAKE_ELEVATOR_HEIGHT_INCH = 30.907161;
    public static double ALGAE_LOW_INTAKE_ENDEFFECTOR_ROTATION_ROTATIONS =
        Units.degreesToRotations(-56.8542103);
    public static double ALGAE_LOW_INTAKE_INTAKE_ROTATION_ROTATIONS =
        Units.degreesToRotations(-26.9162484);

    public static double PROCESSOR_AIM_ELEVATOR_HEIGHT_INCH = 0.0;
    public static double PROCESSOR_AIM_ENDEFFECTOR_ROTATION_ROTATIONS =
        Units.degreesToRotations(-38.3080987);
    public static double PROCESSOR_AIM_INTAKE_ROTATION_ROTATIONS =
        Units.degreesToRotations(-26.9162484);

    public static double BARGE_AIM_ELEVATOR_HEIGHT_INCH = 53.4375;
    public static double BARGE_AIM_FORWARD_ENDEFFECTOR_ROTATION_ROTATIONS =
        Units.degreesToRotations(43.8547133);
    public static double BARGE_AIM_BACKWARD_ENDEFFECTOR_ROTATION_ROTATIONS =
        EndEffectorConstants.MAX_ANGLE_ROTATIONS;
    public static double BARGE_AIM_CENTER_ENDEFFECTOR_ROTATION_ROTATIONS =
        (BARGE_AIM_BACKWARD_ENDEFFECTOR_ROTATION_ROTATIONS
                + BARGE_AIM_FORWARD_ENDEFFECTOR_ROTATION_ROTATIONS)
            / 2;
    public static double BARGE_AIM_INTAKE_ROTATION_ROTATIONS =
        Units.degreesToRotations(-26.9162484);

    public static double ALGAE_GROUND_INTAKE_ELEVATOR_HEIGHT_INCH = 0.0;
    public static double ALGAE_GROUND_INTAKE_ENDEFFECTOR_ROTATION_ROTATIONS =
        Units.degreesToRotations(-38.3080987);
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
