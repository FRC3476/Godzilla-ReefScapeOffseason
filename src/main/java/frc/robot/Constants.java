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

import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecond;

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
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.drive.CommandSwerveDrivetrain;
import frc.robot.subsystems.drive.CompTunerConstants;
import frc.robot.subsystems.drive.SimTunerConstants;
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

  public static final double kSteerJoystickDeadband = 0.012;
  public static final double kRobotMassKg = Units.lbsToKilograms(147.92);
  public static final double kRobotMomentOfInertia = 2 * 9.38; // kg * m^2
  public static final double kCOGHeightMeters = Units.inchesToMeters(0.0);

  public static double kAlignOffset = 0.5;
  public static double kAlignOffsetFB = 0.0;

  // ====================Drive (0_ and 1_)====================
  public static class DriveConstants {
    public static final boolean useMapleSim = false;
    public static final double kDriveMaxSpeed = 3.65;

    public static final double DRIVE_BASE_RADIUS =
        Math.max(
            Math.max(
                Math.hypot(TunerConstants.FrontLeft.LocationX, TunerConstants.FrontLeft.LocationY),
                Math.hypot(
                    TunerConstants.FrontRight.LocationX, TunerConstants.FrontRight.LocationY)),
            Math.max(
                Math.hypot(TunerConstants.BackLeft.LocationX, TunerConstants.BackLeft.LocationY),
                Math.hypot(
                    TunerConstants.BackRight.LocationX, TunerConstants.BackRight.LocationY)));

    // Acceleration limits
    // Large numnbers so they don't do anything.
    public static final double MAX_TRANSLATIONAL_ACCEL = 3476.0; // m/s/s
    public static final double MAX_ROTATIONAL_ACCEL = 3476.0; // rad/s/s

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
    public static final double SLIP_RAMP_RATE = 0.5; // Volts/Sec
    public static final double SLIP_MAX_VOLTAGE = 3476.0; // Volts
    public static final double SLIP_VELOCITY_THRESHOLD =
        3476.0; // Velocity derivative indicating wheels started spinning
    public static final double SLIP_MIN_CURRENT_THRESHOLD = 0.0; // Minimum current threshold

    public static final double AUTO_ALIGN_NORM_TOLERANCE = 0.03;

    public static final double DRIVE_TO_POSE_KP = 6.0;
    public static final double DRIVE_TO_POSE_KI = 0.0;
    public static final double DRIVE_TO_POSE_KD = 0.4;

    public static final double ANGLE_KP = 5.0;
    public static final double ANGLE_KD = 0.4;
    public static final double ANGLE_MAX_ACCELERATION = 15.0;

    public static final double kMaxAccelerationMetersPerSecondSquared = 3.0;
    public static final double kMaxXAccelerationMetersPerSecondSquared = 10.0;
    public static final double kMaxYAccelerationMetersPerSecondSquared = 10.0;
    public static final double kDriveMaxAngularRate = 8.2;
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = 20.0;
    public static final double kHeadingControllerP = 8.0;
    public static final double kHeadingControllerI = 0;
    public static final double kHeadingControllerD = 0;
    public static final CommandSwerveDrivetrain kDrivetrain =
        Robot.isSimulation()
            ? SimTunerConstants.createDrivetrain()
            : CompTunerConstants.createDrivetrain();
    public static final double kRobotWeightPounds = 150.0;
    public static final double kBumperLengthInches = 37.5;
    public static final double kBumperWidthInches = 37.75;
    public static final double kWheelCoefficientOfFriction = 1.0;
    public static final int kDriveMotorCount = 1;

    public static final double kDisabledDriveXStdDev = 1.0;
    public static final double kDisabledDriveYStdDev = 1.0;
    public static final double kDisabledDriveRotStdDev = 1.0;

    public static final double kEnabledDriveXStdDev = 0.3;
    public static final double kEnabledDriveYStdDev = 0.3;
    public static final double kEnabledDriveRotStdDev = 0.2;

    public static final double kDrivePitchThresholdRadians = Units.degreesToRadians(10.0);
    public static final double kDriveRollThresholdRadians = Units.degreesToRadians(10.0);

    public static final double AUTO_ALIGN_PERPENDICULAR_OFFSET = 0.63;
    public static final double AUTO_ALIGN_BARGE_FORWARD_PERPENDICULAR_OFFSET = 1.25 - .80;
    public static final double AUTO_ALIGN_BARGE_BACKWARD_PERPENDICULAR_OFFSET = 1.1 - .175;

    public static final double SCORING_MAX_ROLL_RADIANS = Units.degreesToRadians(5);
    public static final double SCORING_MAX_PITCH_RADIANS = Units.degreesToRadians(5);
    public static final double SCORING_MAX_ROLL_VELOCITY_RADPERSEC = Units.degreesToRadians(10);
    public static final double SCORING_MAX_PITCH_VELOCITY_RADPERSEC =
        Units.degreesToRadians(10);
    public static final LinearVelocity SCORING_MAX_LINEAR_VELOCITY =
        MetersPerSecond.of(15.0 / 100);
    public static final AngularVelocity SCORING_MAX_ANGULAR_VELOCITY = DegreesPerSecond.of(7.0);
  }

  public static final class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond = 3.6;
    public static final double kMaxAccelerationMetersPerSecondSquared = 1.74;
    public static final double kMaxAngularSpeedRadiansPerSecond = 6.5;
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = 31.538;

    public static final double kPXYController = 5.0;
    public static final double kPLTEController = 3.0;
    public static final double kPCTEController = 6.0;
    public static final double kPThetaController = 5.0;

    public static final double kTranslationKa = 0.0;
    public static final double kMaxEndPathVelocity = 2.0; // m/s

    public static final double kTriggerTimeBeforeEnd = 0.8; // seconds

    // Constraint for the motion profiled robot angle controller
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
        new TrapezoidProfile.Constraints(
            kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
  }

  // ====================Feeder (2_)====================
  public static class FeederConstants {
    public static final int RIGHT_ID = 20;
    public static final int LEFT_ID = 21;
    public static final int CANRANGE_ID = 22;
    public static final int FRONT_CANRANGE_ID = 23;

    public static final double ROLLER_MOI = 0.001;
    public static final double ROLLER_GEAR_RATIO = 1.0 / 4.0;

    public static final double ROLLER_CURRENT_LIMIT_AMPS = 40;
    public static final double STALLED_CURRENT = 1000.0;
    public static final double STALLED_RPS = 0.0;
    public static final double DEJAM_DURATION_SECONDS = 0.05;
    public static final double DEJAM_DEBOUNCE_SECONDS = 0.1;

    public static final TalonFXConfiguration ROLLER_TALON_CONFIG =
        new TalonFXConfiguration()
            .withMotorOutput(
                new MotorOutputConfigs()
                    .withInverted(InvertedValue.CounterClockwise_Positive)
                    .withNeutralMode(NeutralModeValue.Brake))
            .withCurrentLimits(
                new CurrentLimitsConfigs()
                    .withSupplyCurrentLimitEnable(true)
                    .withSupplyCurrentLimit(ROLLER_CURRENT_LIMIT_AMPS));

    public static final CANrangeConfiguration CANRANGE_CONFIG =
        new CANrangeConfiguration()
            .withFovParams(new FovParamsConfigs().withFOVRangeX(6.75).withFOVRangeY(6.75))
            .withProximityParams(
                new ProximityParamsConfigs()
                    .withProximityThreshold(Units.inchesToMeters(3))
                    .withProximityHysteresis(0.006));

    public static final double FEEDER_IN_VOLTS = 12.0;
    public static final double FEEDER_OUT_VOLTS = -12.0;
    public static final double FEEDER_STOP_VOLTS = 0.0;
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

    // Setpoints
    public static final double PIVOT_TOLERANCE_ROTATIONS = Units.degreesToRotations(1.5);

    // Pivot Positions
    public static final double PIVOT_INTAKE_POSITION =
        Units.degreesToRotations(-40); // Intake down angle
    public static final double PIVOT_UP_POSITION =
        Units.degreesToRotations(104.5837512); // Intake up angle
    public static final double PIVOT_SCORING_POSITION =
        Units.degreesToRotations(65.7874127); // L1 scoring position

    // L1 Blocker Positions
    public static final double L1_BLOCKER_ENGAGED_POSITION = -0.44;
    public static final double L1_BLOCKER_DISENGAGED_POSITION = 0.0;
    public static final double L1_BLOCKER_ENGAGED_STALL_VOLTAGE = -4;
    public static final double L1_BLOCKER_DISENGAGED_STALL_VOLTAGE = 1;
    public static final double L1_BLOCKER_TORQUE_ENGAGE_AMPS = -20;
    public static final double L1_BLOCKER_TORQUE_DISENGAGE_AMPS = 10;

    // Roller Voltages
    public static final double ROLLER_SCORING_OUT_VOLTS = -6.0;

    public enum IntakeState {
      NONE,
      STOW,
      INTAKE,
      REJECT_CORAL,
      REJECT_INTAKE_CORAL,
      REJECT_INTAKE_CORAL_STAGED,
      REJECT_INTAKE_CORAL_HANDOFF,
      IDLE,
      HAND_OFF,
      SCORING,
      SCORING_PREP,
      JAM_DETECTED
    }

    // Gear ratios
    public static final double PIVOT_RTS = 16 * 12 / 10; // X44- (pivot slap down): (61.71 : 1)
    public static final double PIVOT_STM = 54 / 12;
    public static final double PIVOT_GEAR_RATIO = 1.0 / (PIVOT_RTS * PIVOT_STM);

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

    // public static final double ROLLER_MAX_SUPPLY_CURRENT_LIMIT = 40.0; // Amps
    // public static final double L1_MAX_SUPPLY_CURRENT_LIMIT = 40.0; // Amps
    // public static final double PIVOT_MAX_SUPPLY_CURRENT_LIMIT = 40.0; // Amps

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
    public static final double lvl1blockerKP = 80.0; // Proportional gain
    public static final double lvl1blockerKI = 0.0; // Integral gain
    public static final double lvl1blockerKD = 0.0; // Derivative gain

    // Motion constraints
    public static final double lvl1blockerMAX_ACCEL = 1000; // rad/s^2
    public static final double lvl1blockerMAX_VELOCITY = 50; // rad/s
    public static final double lvl1blockerJERK = 1000; // rad/s^3

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

    public static final CANrangeConfiguration CANRANGE_CONFIG =
        new CANrangeConfiguration()
            .withFovParams(new FovParamsConfigs().withFOVRangeX(27).withFOVRangeY(27))
            .withProximityParams(
                new ProximityParamsConfigs()
                    .withProximityThreshold(0.55)
                    .withProximityHysteresis(0.01));
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

    public static final double DRUM_RADIUS_INCHES = 1.128; // Radius of drum/pulley
    public static final double DRUM_RADIUS_METERS =
        Units.inchesToMeters(DRUM_RADIUS_INCHES); // Radius of drum/pulley
    public static final double kGearing = (1.0 / 5.0); // Motor to drum gearing
    public static final double kElevatorUnitToRotorRatio =
        kGearing
            * 2.0
            * DRUM_RADIUS_METERS
            * Math.PI; // Ratio between elevator units (here, meters for sim bot) to motor rotations

    public static final double CARRIAGE_MASS_KG = 1.97312681; // Mass of elevator carriage

    public static final double ELEVATOR_SETPOINT_TOLERANCE_INCH = 0.5;

    public static final double ELEVATOR_MOTOR_TO_SENSOR_RATIO =
        1
            / (kGearing
                * 2.0
                * DRUM_RADIUS_INCHES
                * Math.PI); // Ratio between elevator units (here, inches for real bot) to motor
    // rotations

    public static final double STALLED_CURRENT = 40;
    public static final double STALLED_RPS = 2;
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
                    .withInverted(InvertedValue.CounterClockwise_Positive)
                    .withNeutralMode(NeutralModeValue.Brake))
            .withCurrentLimits(
                new CurrentLimitsConfigs()
                    .withSupplyCurrentLimitEnable(true)
                    .withSupplyCurrentLimit(ELEVATOR_CURRENT_LIMIT_AMPS));

    // ========Elevator Constant Positions========
    public static final double ELEVATOR_ZERO_SETPOINT_INCH = 0.0;
    public static final double ELEVATOR_MAX_SETPOINT_INCH = 55.5; // max height

    // Homing sequence constants
    public static final double ELEVATOR_HOMING_VOLTAGE = -1; // Downward voltage for homing
    public static final double HOMING_TIMEOUT_SECONDS = 6.0; // Max time to allow for homing

    // Coral scoring heights
    public static final double ELEVATOR_L2_AGAINST_REEF_SETPOINT_INCH =
        0; // 8.675; // 14.418111 + 2;
    public static final double ELEVATOR_L2_AWAY_FROM_REEF_SETPOINT_INCH = 7.078988 + 1;
    public static final double ELEVATOR_L1_AGAINST_REEF_FADEAWAY_SETPOINT_INCH = 5;
    public static final double ELEVATOR_L2_AGAINST_REEF_FADEAWAY_SETPOINT_INCH =
        ELEVATOR_L2_AGAINST_REEF_SETPOINT_INCH;
    public static final double ELEVATOR_L3_AGAINST_REEF_SETPOINT_INCH = 24.79; // 30.029785 + 2.5;
    public static final double ELEVATOR_L3_AWAY_FROM_REEF_SETPOINT_INCH = 23.003301 + 1;
    public static final double ELEVATOR_L3_AGAINST_REEF_FADEAWAY_SETPOINT_INCH =
        ELEVATOR_L3_AGAINST_REEF_SETPOINT_INCH - 4;
    public static final double ELEVATOR_L4_AGAINST_REEF_SETPOINT_INCH = ELEVATOR_MAX_SETPOINT_INCH;
    public static final double ELEVATOR_L4_AWAY_FROM_REEF_SETPOINT_INCH =
        ELEVATOR_MAX_SETPOINT_INCH;
    public static final double ELEVATOR_L4_AGAINST_REEF_FADEAWAY_SETPOINT_INCH =
        ELEVATOR_L4_AGAINST_REEF_SETPOINT_INCH;

    // Algae scoring heights
    public static final double ELEVATOR_L2_ALGAE_AGAINST_REEF_SETPOINT_INCH = 30.907161 - 1.5 - 1.5;
    public static final double ELEVATOR_L3_ALGAE_AGAINST_REEF_SETPOINT_INCH = 45.325558 - 1.5;

    // Barge heights
    public static final double ELEVATOR_BARGE_SETPOINT_INCH = ELEVATOR_MAX_SETPOINT_INCH;

    public static final double MIN_HEIGHT_METERS =
        Units.inchesToMeters(ELEVATOR_ZERO_SETPOINT_INCH); // Minimum elevator height
    public static final double MAX_HEIGHT_METERS =
        Units.inchesToMeters(ELEVATOR_MAX_SETPOINT_INCH); // Maximum elevator height
  }

  // ====================End Effector (5_)====================
  public static class EndEffectorConstants {
    public static final int pivotID = 50;
    public static final int rollerID = 51;
    public static final int FIRST_CORAL_CANRANGE_ID = 52;
    public static final int SECOND_CORAL_CANRANGE_ID = 53;
    public static final int PIVOT_CANCODER_ID = 54;

    public static final double ROLLER_MOI = 0.001;

    public static final double Tunable_PIVOT_kP = 60;
    public static final double Tunable_PIVOT_kI = 0;
    public static final double Tunable_PIVOT_kD = 3.5;
    public static final double Tunable_PIVOT_kG = 0.9;
    public static final double Tunable_PIVOT_kS = 0.35;
    public static final double Tunable_PIVOT_kA = 0.19;
    public static final double Tunable_PIVOT_kV = 5;

    public static final double Tunable_PIVOT_Velo = 1000;
    public static final double Tunable_PIVOT_Accel = 9;
    public static final double Tunable_PIVOT_Jerk = 1000;

    public static final double PIVOT_CURRENT_LIMIT_AMPS = 40;

    public static final double ROLLER_CURRENT_LIMIT_AMPS = 40;

    public static final double ALGAE_GEAR_RATIO = 1.0 / 12.22;
    public static final double CORAL_GEAR_RATIO = 1.0 / 6.11;

    public static final double PIVOT_RTS = 10;
    public static final double PIVOT_STM = 4;
    public static final double PIVOT_GEAR_RATIO = PIVOT_RTS * PIVOT_STM;

    public static final double ROLLER_STALLED_CURRENT = 40;
    public static final double ROLLER_STALLED_RPS = 10;

    public static final double CLAW_HOLD_ALGAE_AMPS = 70.0;
    public static final double PIVOT_TOLERANCE_ROTATIONS = Units.degreesToRotations(2.5);

    // ========End Effector Constant Positions========
    // Standardized angle constants with RADIAN suffix
    public static final double MAX_ANGLE_ROTATIONS = Units.degreesToRotations(119.8473749);
    public static final double MIN_ANGLE_ROTATIONS = Units.degreesToRotations(-92.16);
    public static final double MAX_SAFE_ANGLE_ROTATIONS = .155; // old value 53.9126895
    public static final double MIN_SAFE_ANGLE_ROTATIONS = -.169; // old value -61.1115004
    public static final double L2_L3_FADEAWAY_ANGLE_ROTATIONS =
        Units.degreesToRotations(17.7998883 + 5);

    // Pivot positions in rotations
    public static final double IDLE_ANGLE_ROTATIONS = MIN_ANGLE_ROTATIONS;
    public static final double ALGAE_GROUND_ANGLE_ROTATIONS = -0.121337890625 - 0.01;
    public static final double ALGAE_IDLE_ANGLE_ROTATIONS = Units.degreesToRotations(-38.3080987);
    public static final double PROCESSOR_ANGLE_ROTATIONS = -0.033447265625;
    public static final double L1_FADEAWAY_ANGLE_ROTATIONS = -.21;
    public static final double L2_AGAINST_REEF_ANGLE_ROTATIONS =
        MAX_SAFE_ANGLE_ROTATIONS - Units.degreesToRotations(7);
    public static final double L3_AGAINST_REEF_ANGLE_ROTATIONS =
        Units.degreesToRotations(17.7998883); // Units.degreesToRotations(-16.3769186);
    public static final double L2_L3_AWAY_FROM_REEF_ANGLE_ROTATIONS =
        Units.degreesToRotations(17.7998883);
    public static final double L4_AGAINST_REEF_ANGLE_ROTATIONS = Units.degreesToRotations(2);
    public static final double L4_AWAY_FROM_REEF_ANGLE_ROTATIONS = L4_AGAINST_REEF_ANGLE_ROTATIONS;
    public static final double ALGAE_REMOVAL_ANGLE_ROTATIONS =
        Units.degreesToRotations(-56.8542103);
    public static final double BARGE_FORWARD_ANGLE_ROTATIONS = Units.degreesToRotations(43.8547133);
    public static final double BARGE_BACKWARD_ANGLE_ROTATIONS = MAX_ANGLE_ROTATIONS;
    public static final double PIVOT_ABSOLUTE_ENCODER_OFFSET = 0.305908;
    public static final double PIVOT_CLIMB_SAFE_ROTATIONS =
        -.185; // Units.degreesToRotations(-82.8);

    public static final TalonFXConfiguration PIVOT_TALON_CONFIG =
        new TalonFXConfiguration()
            .withSlot0(
                new Slot0Configs()
                    .withKP(Tunable_PIVOT_kP)
                    .withKI(Tunable_PIVOT_kI)
                    .withKD(Tunable_PIVOT_kD)
                    .withKG(Tunable_PIVOT_kG)
                    .withKA(Tunable_PIVOT_kA)
                    .withKV(Tunable_PIVOT_kV)
                    .withGravityType(GravityTypeValue.Arm_Cosine))
            .withSlot1(
                new Slot1Configs()
                    .withKP(Tunable_PIVOT_kP * 4)
                    .withKI(Tunable_PIVOT_kI)
                    .withKD(Tunable_PIVOT_kD)
                    .withKG(Tunable_PIVOT_kG)
                    .withKA(Tunable_PIVOT_kA)
                    .withKV(Tunable_PIVOT_kV)
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
                    .withSupplyCurrentLimitEnable(true)
                    .withSupplyCurrentLimit(PIVOT_CURRENT_LIMIT_AMPS));

    public static final CANcoderConfiguration PIVOT_CANCODER_CONFIG =
        new CANcoderConfiguration()
            .withMagnetSensor(
                new MagnetSensorConfigs()
                    .withSensorDirection(SensorDirectionValue.CounterClockwise_Positive)
                    .withAbsoluteSensorDiscontinuityPoint(0.5));

    public static final TalonFXConfiguration ROLLER_TALON_CONFIG =
        new TalonFXConfiguration()
            .withMotorOutput(
                new MotorOutputConfigs()
                    .withInverted(InvertedValue.Clockwise_Positive)
                    .withNeutralMode(NeutralModeValue.Brake))
            .withCurrentLimits(
                new CurrentLimitsConfigs()
                    .withSupplyCurrentLimitEnable(true)
                    .withSupplyCurrentLimit(ROLLER_CURRENT_LIMIT_AMPS));

    public static final CANrangeConfiguration CANRANGE_CONFIG =
        new CANrangeConfiguration()
            .withFovParams(new FovParamsConfigs().withFOVRangeX(12.75).withFOVRangeY(12.75))
            .withProximityParams(
                new ProximityParamsConfigs()
                    .withProximityThreshold(Units.inchesToMeters(3))
                    .withProximityHysteresis(0.006));

    public enum ClawState {
      NONE,
      IDLE,
      INTAKING_CORAL,
      HOLDING_CORAL,
      SCORING,
      SCORING_L1,
      SCORING_ALGAE,
      ALGAE
    }

    // Roller Voltages
    public static final double ROLLER_INTAKE_CORAL_VOLTS = 7;
    public static final double ROLLER_SCORING_VOLTS = -12;
    public static final double ROLLER_SCORING_L1_VOLTS = 6;
    public static final double ROLLER_SCORING_ALGAE_VOLTS = -5;
    public static final double ROLLER_HOLDING_CORAL_VOLTS = 3;

    // Reef Collision Avoidance
    public static final double FULLY_EXTENDED_DISTANCE_METERS =
        Units.inchesToMeters(
            20); // distance from the center of the robot to the end of the end effector
    public static final double MIN_STOW_CLEARANCE_METERS =
        Units.inchesToMeters(
            20); // area around that point that would hit something on the end effector
  }

  // ====================Climb (6_)====================
  public static class ClimbConstants {

    public static final double reduction = (1 / 23.11);
    public static final double climbMOI = 0.01;

    public static final int ID = 60;
    public static final int rollerID = 61;

    public static final double ROLLER_MOI = 0.001;
    public static final double ROLLER_GEAR_RATIO = 4; // TODO : update with true value

    public static final double CLIMB_DEPLOY_POSITION = 87.5;
    public static final double CLIMB_CLIMB_POSITION = 215;
    public static final double CLIMB_DEPLOY_VOLTAGE = 12;
    public static final double CLIMB_CLIMB_VOLTAGE = 12;
    public static final double STALL_AMPS = 1000.0;
    public static final double STALL_VELOCITY = 0.0;
    public static final double CLIMB_ANGLE_SNAP = 109.69;

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

    public static final double ROLLER_HOLD_CAGE_AMPS = -60.0; // TODO fine-adjust
    public static final double ROLLER_BACKOUT_VOLTS = 0.5; // TODO fine-adjust
    public static final double ROLLER_CURRENT_LIMIT_AMPS = 80.0;
    public static final double ROLLER_STALLED_RPS = 2; // TODO fine-adjust
    public static final double ROLLER_STALLED_CURRENT = 50.0; // TODO fine-adjust

    public static final TalonFXConfiguration ROLLER_TALON_CONFIG =
        new TalonFXConfiguration()
            .withMotorOutput(
                new MotorOutputConfigs()
                    .withInverted(InvertedValue.CounterClockwise_Positive)
                    .withNeutralMode(NeutralModeValue.Brake))
            .withCurrentLimits(
                new CurrentLimitsConfigs()
                    .withSupplyCurrentLimitEnable(true)
                    .withSupplyCurrentLimit(ClimbConstants.ROLLER_CURRENT_LIMIT_AMPS));
  }

  // ====================LED (8_)====================
  public static final class LEDConstants {
    public static final int ID = 19; // 80 not allowed, max ID is 62
    public static final int kNonCandleLEDCount = 10;
    public static final int kCandleLEDCount = 8;
    public static final int kMaxLEDCount = kNonCandleLEDCount + kCandleLEDCount;
    public static final double kLowBatteryThresholdVolts = 12.3;
  }

  // ====================Vision Constants====================
  public static class VisionConstants {
    public static final String DETECTION_LIMELIGHT = "limelight-intake";
    public static final AprilTagFieldLayout fieldLayout =
        AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);
    public static final AprilTagFieldLayout kAprilTagLayout =
        AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);

    // Camera A (left side)
    public static final double kCameraAPitchDegrees = 11.57;
    public static final double kCameraAPitchRads = Units.degreesToRadians(kCameraAPitchDegrees);
    public static final double kCameraAHeightOffGroundMeters = Units.inchesToMeters(8.580998);
    public static final String kLimelightATableName = "limelight-left";
    public static final double kRobotToCameraAForward = Units.inchesToMeters(-11.422523);
    public static final double kRobotToCameraASide = Units.inchesToMeters(-10.365637);
    public static final Rotation2d kCameraAYawOffset = Rotation2d.fromDegrees(-148.64);
    public static final double kCameraARollDegrees = -9.97;
    public static final Transform2d kRobotToCameraA =
        new Transform2d(
            new Translation2d(kRobotToCameraAForward, kRobotToCameraASide), kCameraAYawOffset);

    // Camera B (right side)
    public static final double kCameraBPitchDegrees = 11.57;
    public static final double kCameraBPitchRads = Units.degreesToRadians(kCameraBPitchDegrees);
    public static final double kCameraBHeightOffGroundMeters = Units.inchesToMeters(8.580998);
    public static final String kLimelightBTableName = "limelight-right";
    public static final double kRobotToCameraBForward = Units.inchesToMeters(-11.422523);
    public static final double kRobotToCameraBSide = Units.inchesToMeters(10.365637);
    public static final Rotation2d kCameraBYawOffset = Rotation2d.fromDegrees(148.64);
    public static final double kCameraBRollDegrees = 9.97;
    public static final Transform2d kRobotToCameraB =
        new Transform2d(
            new Translation2d(kRobotToCameraBForward, kRobotToCameraBSide), kCameraBYawOffset);

    // April Tags

    public static final int[] kAllowedTagIDs = {17, 18, 19, 20, 21, 22, 6, 7, 8, 9, 10, 11};

    public static final AprilTagFieldLayout kAprilTagLayoutReefsOnly =
        new AprilTagFieldLayout(
            kAprilTagLayout.getTags().stream()
                .filter(tag -> Arrays.stream(kAllowedTagIDs).anyMatch(element -> element == tag.ID))
                .toList(),
            kAprilTagLayout.getFieldLength(),
            kAprilTagLayout.getFieldWidth());

    // Vision processing constants
    public static final double kDefaultAmbiguityThreshold = 0.19;
    public static final double kDefaultYawDiffThreshold = 5.0;
    public static final double kTagAreaThresholdForYawCheck = 2.0;
    public static final double kTagMinAreaForSingleTagMegatag = 1.0;
    public static final double kTagMinAreaForMultipleTagMegatag = 0.4;
    public static final double kDefaultZThreshold = 0.5;
    public static final double kDefaultNormThreshold = 1.0;
    public static final double kMinAmbiguityToFlip = 0.08;
    public static final double kXStdDevCoefficent = 0.3;
    public static final double kYStdDevCoefficent = 0.3;

    public static final double thetaStdDevCoefficient = 3476.0;

    public static final double kCameraHorizontalFOVDegrees = 81.0;
    public static final double kCameraVerticalFOVDegrees = 55.0;
    public static final int kCameraImageWidth = 1280;
    public static final int kCameraImageHeight = 800;
    public static final double kScoringConfidenceThreshold = 0.7;
    // NetworkTables constants
    public static final String kBoundingBoxTableName = "BoundingBoxes";

    // Large variance used to downweight unreliable vision measurements
    public static final double kLargeVariance = 1e6;
    // Standard deviation constants
    public static final int kMegatag1XStdDevIndex = 0;
    public static final int kMegatag1YStdDevIndex = 1;
    public static final int kMegatag1YawStdDevIndex = 5;
    // Standard deviation array indices for Megatag2
    public static final int kMegatag2XStdDevIndex = 6;
    public static final int kMegatag2YStdDevIndex = 7;
    public static final int kMegatag2YawStdDevIndex = 11;
    // Validation constants
    public static final int kMinFiducialCount = 1;
    public static final int kExpectedStdDevArrayLength = 12;
  }

  public static class SuperstructureConstants {

    public static double CLIMB_ENDEFFECTOR_SAFE_ROTATIONS =
        EndEffectorConstants.PIVOT_CLIMB_SAFE_ROTATIONS;

    public static double LOW_IN_SAFE_ELEVATOR_HEIGHT_INCHES = 12.9;
    public static double HIGH_IN_SAFE_ELEVATOR_HEIGHT_INCHES = 45.7;

    public static double STOW_ELEVATOR_HEIGHT_INCH = ElevatorConstants.ELEVATOR_ZERO_SETPOINT_INCH;
    public static double STOW_ENDEFFECTOR_ROTATION_ROTATIONS =
        EndEffectorConstants.MIN_ANGLE_ROTATIONS;

    public static double STOW_CORAL_ELEVATOR_HEIGHT_INCH =
        ElevatorConstants.ELEVATOR_ZERO_SETPOINT_INCH;
    public static double STOW_CORAL_ENDEFFECTOR_ROTATION_ROTATIONS =
        EndEffectorConstants.MIN_ANGLE_ROTATIONS;

    public static double STOW_ALGAE_ELEVATOR_HEIGHT_INCH =
        ElevatorConstants.ELEVATOR_ZERO_SETPOINT_INCH;
    public static double STOW_ALGAE_ENDEFFECTOR_ROTATION_ROTATIONS = 0;

    public static double INTAKE_CORAL_ELEVATOR_HEIGHT_INCH =
        ElevatorConstants.ELEVATOR_ZERO_SETPOINT_INCH;
    public static double INTAKE_CORAL_ENDEFFECTOR_ROTATION_ROTATIONS =
        EndEffectorConstants.MIN_ANGLE_ROTATIONS;

    public static double INTAKE_CORAL_L1_ELEVATOR_HEIGHT_INCH =
        ElevatorConstants.ELEVATOR_ZERO_SETPOINT_INCH;
    public static double INTAKE_CORAL_L1_ENDEFFECTOR_ROTATION_ROTATIONS =
        EndEffectorConstants.MIN_ANGLE_ROTATIONS;

    public static double FEED_ELEVATOR_HEIGHT_INCH = ElevatorConstants.ELEVATOR_ZERO_SETPOINT_INCH;
    public static double FEED_ENDEFFECTOR_ROTATION_ROTATIONS =
        EndEffectorConstants.MIN_ANGLE_ROTATIONS;

    public static double CORAL_STUCK_UNDER_FEEDER_ELEVATOR_HEIGHT_INCH =
        ElevatorConstants.ELEVATOR_ZERO_SETPOINT_INCH;
    public static double CORAL_STUCK_UNDER_FEEDER_ENDEFFECTOR_ROTATION_ROTATIONS =
        EndEffectorConstants.MAX_SAFE_ANGLE_ROTATIONS;

    public static double L1_AIM_ELEVATOR_HEIGHT_INCH =
        ElevatorConstants.ELEVATOR_ZERO_SETPOINT_INCH;
    public static double L1_AIM_ENDEFFECTOR_ROTATION_ROTATIONS =
        EndEffectorConstants.MIN_ANGLE_ROTATIONS + .02;
    public static double L2_AIM_ELEVATOR_HEIGHT_INCH =
        ElevatorConstants.ELEVATOR_L2_AGAINST_REEF_SETPOINT_INCH;
    public static double L2_AIM_AWAY_FROM_REEF_ELEVATOR_HEIGHT_INCH =
        ElevatorConstants.ELEVATOR_L2_AWAY_FROM_REEF_SETPOINT_INCH;
    public static double L2_AIM_ENDEFFECTOR_ROTATION_ROTATIONS =
        EndEffectorConstants.L2_AGAINST_REEF_ANGLE_ROTATIONS;
    public static double L2_AIM_AWAY_FROM_REEF_ENDEFFECTOR_ROTATION_ROTATIONS =
        EndEffectorConstants.L2_L3_AWAY_FROM_REEF_ANGLE_ROTATIONS;

    public static double L3_AIM_ELEVATOR_HEIGHT_INCH =
        ElevatorConstants.ELEVATOR_L3_AGAINST_REEF_SETPOINT_INCH;
    public static double L3_AIM_AWAY_FROM_REEF_ELEVATOR_HEIGHT_INCH =
        ElevatorConstants.ELEVATOR_L3_AWAY_FROM_REEF_SETPOINT_INCH;
    public static double L3_AIM_ENDEFFECTOR_ROTATION_ROTATIONS =
        EndEffectorConstants.L3_AGAINST_REEF_ANGLE_ROTATIONS;
    public static double L3_AIM_AWAY_FROM_REEF_ENDEFFECTOR_ROTATION_ROTATIONS =
        EndEffectorConstants.L2_L3_AWAY_FROM_REEF_ANGLE_ROTATIONS;

    public static double L4_AIM_ELEVATOR_HEIGHT_INCH =
        ElevatorConstants.ELEVATOR_L4_AGAINST_REEF_SETPOINT_INCH;
    public static double L4_AIM_AWAY_FROM_REEF_ELEVATOR_HEIGHT_INCH =
        ElevatorConstants.ELEVATOR_L4_AWAY_FROM_REEF_SETPOINT_INCH;
    public static double L4_AIM_ENDEFFECTOR_ROTATION_ROTATIONS =
        EndEffectorConstants.L4_AGAINST_REEF_ANGLE_ROTATIONS;
    public static double L4_AIM_AWAY_FROM_REEF_ENDEFFECTOR_ROTATION_ROTATIONS =
        EndEffectorConstants.L4_AWAY_FROM_REEF_ANGLE_ROTATIONS;

    public static double L1_SCORE_ELEVATOR_HEIGHT_INCH =
        ElevatorConstants.ELEVATOR_ZERO_SETPOINT_INCH;
    public static double L1_SCORE_ENDEFFECTOR_ROTATION_ROTATIONS =
        EndEffectorConstants.MIN_ANGLE_ROTATIONS;

    public static double L2_SCORE_ELEVATOR_HEIGHT_INCH =
        ElevatorConstants.ELEVATOR_L2_AGAINST_REEF_SETPOINT_INCH;
    // Away from reef 7.078988deg
    public static double L2_SCORE_ENDEFFECTOR_ROTATION_ROTATIONS =
        EndEffectorConstants.L2_AGAINST_REEF_ANGLE_ROTATIONS;
    // Away from reef 17.7998883deg

    public static double L3_SCORE_ELEVATOR_HEIGHT_INCH =
        ElevatorConstants.ELEVATOR_L3_AGAINST_REEF_SETPOINT_INCH;
    // Away from reef 23.003301deg
    public static double L3_SCORE_ENDEFFECTOR_ROTATION_ROTATIONS =
        EndEffectorConstants.L3_AGAINST_REEF_ANGLE_ROTATIONS;
    // Away from reef 17.7998883deg

    public static double L4_SCORE_ELEVATOR_HEIGHT_INCH =
        ElevatorConstants.ELEVATOR_L4_AGAINST_REEF_SETPOINT_INCH;
    public static double L4_SCORE_ENDEFFECTOR_ROTATION_ROTATIONS =
        EndEffectorConstants.L4_AGAINST_REEF_ANGLE_ROTATIONS; // Away from reef (no against)

    public static double L1_FADEAWAY_ELEVATOR_HEIGHT_INCH =
        ElevatorConstants.ELEVATOR_L1_AGAINST_REEF_FADEAWAY_SETPOINT_INCH;
    public static double L1_FADEAWAY_ENDEFFECTOR_ROTATION_ROTATIONS =
        EndEffectorConstants.L1_FADEAWAY_ANGLE_ROTATIONS;

    public static double L2_FADEAWAY_ELEVATOR_HEIGHT_INCH =
        ElevatorConstants.ELEVATOR_L2_AGAINST_REEF_FADEAWAY_SETPOINT_INCH;
    public static double L2_FADEAWAY_ENDEFFECTOR_ROTATION_ROTATIONS =
        EndEffectorConstants.L2_L3_FADEAWAY_ANGLE_ROTATIONS;

    public static double L3_FADEAWAY_ELEVATOR_HEIGHT_INCH =
        ElevatorConstants.ELEVATOR_L3_AGAINST_REEF_FADEAWAY_SETPOINT_INCH;
    public static double L3_FADEAWAY_ENDEFFECTOR_ROTATION_ROTATIONS =
        EndEffectorConstants.L2_L3_FADEAWAY_ANGLE_ROTATIONS;

    public static double L4_FADEAWAY_ELEVATOR_HEIGHT_INCH =
        ElevatorConstants.ELEVATOR_L4_AGAINST_REEF_FADEAWAY_SETPOINT_INCH;
    public static double L4_FADEAWAY_ENDEFFECTOR_ROTATION_ROTATIONS =
        EndEffectorConstants.MAX_SAFE_ANGLE_ROTATIONS;

    public static double ALGAE_HIGH_INTAKE_ELEVATOR_HEIGHT_INCH =
        ElevatorConstants.ELEVATOR_L3_ALGAE_AGAINST_REEF_SETPOINT_INCH;
    public static double ALGAE_HIGH_INTAKE_ENDEFFECTOR_ROTATION_ROTATIONS =
        EndEffectorConstants.ALGAE_REMOVAL_ANGLE_ROTATIONS;

    public static double ALGAE_LOW_INTAKE_ELEVATOR_HEIGHT_INCH =
        ElevatorConstants.ELEVATOR_L2_ALGAE_AGAINST_REEF_SETPOINT_INCH;
    public static double ALGAE_LOW_INTAKE_ENDEFFECTOR_ROTATION_ROTATIONS =
        EndEffectorConstants.ALGAE_REMOVAL_ANGLE_ROTATIONS;

    public static double PROCESSOR_AIM_ELEVATOR_HEIGHT_INCH =
        ElevatorConstants.ELEVATOR_ZERO_SETPOINT_INCH;
    public static double PROCESSOR_AIM_ENDEFFECTOR_ROTATION_ROTATIONS =
        EndEffectorConstants.PROCESSOR_ANGLE_ROTATIONS;

    public static double BARGE_AIM_ELEVATOR_HEIGHT_INCH =
        ElevatorConstants.ELEVATOR_BARGE_SETPOINT_INCH;
    public static double BARGE_AIM_FORWARD_ENDEFFECTOR_ROTATION_ROTATIONS =
        EndEffectorConstants.BARGE_FORWARD_ANGLE_ROTATIONS;
    public static double BARGE_AIM_BACKWARD_ENDEFFECTOR_ROTATION_ROTATIONS =
        EndEffectorConstants.BARGE_BACKWARD_ANGLE_ROTATIONS;
    public static double BARGE_AIM_CENTER_ENDEFFECTOR_ROTATION_ROTATIONS =
        (BARGE_AIM_BACKWARD_ENDEFFECTOR_ROTATION_ROTATIONS
                + BARGE_AIM_FORWARD_ENDEFFECTOR_ROTATION_ROTATIONS)
            / 2;

    public static double ALGAE_GROUND_INTAKE_ELEVATOR_HEIGHT_INCH =
        ElevatorConstants.ELEVATOR_ZERO_SETPOINT_INCH;
    public static double ALGAE_GROUND_INTAKE_ENDEFFECTOR_ROTATION_ROTATIONS =
        EndEffectorConstants.ALGAE_GROUND_ANGLE_ROTATIONS;
  }
}
