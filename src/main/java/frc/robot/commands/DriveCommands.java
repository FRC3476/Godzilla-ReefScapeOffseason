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

package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.RobotState;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.superstructure.CoralStateTracker;
import frc.robot.subsystems.superstructure.CoralStateTracker.CoralPosition;
import frc.robot.subsystems.vision.Vision;
import frc.robot.util.Util;
import java.text.DecimalFormat;
import java.text.NumberFormat;
import java.util.LinkedList;
import java.util.List;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class DriveCommands {
  private static final double DEADBAND = 0.025;

  private DriveCommands() {}

  private static Translation2d getLinearVelocityFromJoysticks(double x, double y) {
    // Apply deadband
    double linearMagnitude = MathUtil.applyDeadband(Math.hypot(x, y), DEADBAND);
    Rotation2d linearDirection = new Rotation2d(Math.atan2(y, x));

    // Square magnitude for more precise control
    linearMagnitude = linearMagnitude * linearMagnitude;

    // Return new linear velocity
    return new Pose2d(Translation2d.kZero, linearDirection)
        .transformBy(new Transform2d(linearMagnitude, 0.0, Rotation2d.kZero))
        .getTranslation();
  }

  /**
   * Field relative drive command using two joysticks (controlling linear and angular velocities).
   */
  public static Command joystickDrive(
      DriveSubsystem drive,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      DoubleSupplier omegaSupplier) {

    SwerveRequest.FieldCentric fieldCentricReq =
        new SwerveRequest.FieldCentric()
            // .withDeadband(
            //     Constants.DriveConstants.kDriveMaxSpeed * 0.025) // Add a 5% deadband in open
            // loop
            // .withRotationalDeadband(
            //     Constants.DriveConstants.kDriveMaxAngularRate * Constants.kSteerJoystickDeadband)
            .withDriveRequestType(SwerveModule.DriveRequestType.Velocity);

    return Commands.run(
        () -> {
          // Square linear values for more precise control
          double xJoy = xSupplier.getAsDouble();
          double yJoy = ySupplier.getAsDouble();
          xJoy = Util.handleDeadband(xJoy, 0.05);
          yJoy = Util.handleDeadband(yJoy, 0.05);
          xJoy = Math.copySign(xJoy * xJoy, xJoy);
          yJoy = Math.copySign(yJoy * yJoy, yJoy);

          //   // Apply rotation deadband
          //   double omega = MathUtil.applyDeadband(omegaSupplier.getAsDouble(), DEADBAND);

          // Square rotation value for more precise control
          double omega = omegaSupplier.getAsDouble();
          omega = Util.handleDeadband(omega, 0.1);
          omega = Math.copySign(omega * omega, omega);

          // Convert to field relative speeds & send command
          drive.setControl(
              fieldCentricReq
                  .withVelocityX(xJoy * Constants.DriveConstants.kDriveMaxSpeed)
                  .withVelocityY(yJoy * Constants.DriveConstants.kDriveMaxSpeed)
                  .withRotationalRate(omega * Constants.DriveConstants.kDriveMaxAngularRate));
        },
        drive);
  }

  public static Command StopDriveTrain(DriveSubsystem driveSubsystem) {
    return Commands.run(
        () ->
            driveSubsystem.setControl(
                new SwerveRequest.FieldCentric()
                    .withVelocityX(0.0)
                    .withVelocityY(0.0)
                    .withRotationalRate(0.0)));
  }

  /**
   * Field relative drive command using joystick for linear control and PID for angular control.
   * Possible use cases include snapping to an angle, aiming at a vision target, or controlling
   * absolute rotation with a joystick.
   */
  public static Command driveAtAngle(
      DriveSubsystem drive,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      Supplier<Rotation2d> rotationSupplier) {

    // Construct command
    return Commands.run(
        () -> {
          double xJoy = xSupplier.getAsDouble();
          double yJoy = ySupplier.getAsDouble();
          xJoy = Util.handleDeadband(xJoy, 0.05);
          yJoy = Util.handleDeadband(yJoy, 0.05);
          xJoy = Math.copySign(xJoy * xJoy, xJoy);
          yJoy = Math.copySign(yJoy * yJoy, yJoy);

          // Create PID controller
          ProfiledPIDController angleController =
              new ProfiledPIDController(
                  DriveConstants.ANGLE_KP,
                  0.0,
                  DriveConstants.ANGLE_KD,
                  new TrapezoidProfile.Constraints(
                      DriveConstants.kDriveMaxAngularRate, DriveConstants.ANGLE_MAX_ACCELERATION));
          angleController.enableContinuousInput(-Math.PI, Math.PI);

          SwerveRequest.FieldCentricFacingAngle facingAngle =
              new SwerveRequest.FieldCentricFacingAngle()
                  .withDriveRequestType(DriveRequestType.Velocity)
                  .withHeadingPID(DriveConstants.ANGLE_KP, 0.0, DriveConstants.ANGLE_KD)
                  .withDesaturateWheelSpeeds(true);

          Rotation2d rotTarget = Rotation2d.kZero;

          var alliance = DriverStation.getAlliance();
          if (alliance.isPresent()) {
            if (alliance.get() == Alliance.Red) {
              rotTarget = Rotation2d.kPi;
            }
          }

          drive.setControl(
              facingAngle
                  .withVelocityX(xJoy * Constants.DriveConstants.kDriveMaxSpeed)
                  .withVelocityY(yJoy * Constants.DriveConstants.kDriveMaxSpeed)
                  .withTargetDirection(rotationSupplier.get().rotateBy(rotTarget)));
        },
        drive);
  }

  // drive using object detection for coral

  public static Command driveToCoral(DriveSubsystem drive, Vision vision) {
    boolean isFlipped =
        DriverStation.getAlliance().isPresent()
            && DriverStation.getAlliance().get() == Alliance.Red;
    DoubleSupplier xSupplier =
        () ->
            isFlipped
                ? (vision.getCoralTy() * 0.15)
                    * Rotation2d.fromDegrees(vision.getCoralTx()).getCos()
                : (vision.getCoralTy() * 0.15)
                    * Rotation2d.fromDegrees(vision.getCoralTx()).getCos();
    DoubleSupplier ySupplier = () -> -3 * Rotation2d.fromDegrees(vision.getCoralTx()).getSin();
    Supplier<Rotation2d> rotSupplier =
        () ->
            Rotation2d.fromDegrees(
                RobotState.getGlobalPose().getRotation().getDegrees()
                    - vision.getCoralTx()
                    + (isFlipped ? 180 : 0));
    return driveAtAngle(drive, xSupplier, ySupplier, rotSupplier)
        .onlyWhile(() -> CoralStateTracker.getCurrentPosition() == CoralPosition.NONE);
  }

  /**
   * Measures the velocity feedforward constants for the drive motors.
   *
   * <p>This command should only be used in voltage control mode.
   */
  public static Command feedforwardCharacterization(DriveSubsystem drive) {
    List<Double> velocitySamples = new LinkedList<>();
    List<Double> voltageSamples = new LinkedList<>();
    Timer timer = new Timer();

    return Commands.sequence(
        // Reset data
        Commands.runOnce(
            () -> {
              velocitySamples.clear();
              voltageSamples.clear();
            }),

        // Allow modules to orient
        Commands.run(
                () -> {
                  drive.runCharacterization(0.0);
                },
                drive)
            .withTimeout(Constants.DriveConstants.FF_START_DELAY),

        // Start timer
        Commands.runOnce(timer::restart),

        // Accelerate and gather data
        Commands.run(
                () -> {
                  double voltage = timer.get() * Constants.DriveConstants.FF_RAMP_RATE;
                  drive.runCharacterization(voltage);
                  velocitySamples.add(drive.getFFCharacterizationVelocity());
                  voltageSamples.add(voltage);
                },
                drive)

            // When cancelled, calculate and print results
            .finallyDo(
                () -> {
                  int n = velocitySamples.size();
                  double sumX = 0.0;
                  double sumY = 0.0;
                  double sumXY = 0.0;
                  double sumX2 = 0.0;
                  for (int i = 0; i < n; i++) {
                    sumX += velocitySamples.get(i);
                    sumY += voltageSamples.get(i);
                    sumXY += velocitySamples.get(i) * voltageSamples.get(i);
                    sumX2 += velocitySamples.get(i) * velocitySamples.get(i);
                  }
                  double kS = (sumY * sumX2 - sumX * sumXY) / (n * sumX2 - sumX * sumX);
                  double kV = (n * sumXY - sumX * sumY) / (n * sumX2 - sumX * sumX);

                  NumberFormat formatter = new DecimalFormat("#0.00000");
                  System.out.println("********** Drive FF Characterization Results **********");
                  System.out.println("\tkS: " + formatter.format(kS));
                  System.out.println("\tkV: " + formatter.format(kV));
                }));
  }

  // /** Measures the wheel slip current by driving against a wall (wall test). */
  // public static Command slipCurrentCharacterization(Drive drive) {
  //   List<Double> currentSamples = new LinkedList<>();
  //   List<Double> velocitySamples = new LinkedList<>();
  //   List<Double> voltageSamples = new LinkedList<>();
  //   Timer timer = new Timer();

  //   return Commands.sequence(
  //       // Reset data
  //       Commands.runOnce(
  //           () -> {
  //             currentSamples.clear();
  //             velocitySamples.clear();
  //             voltageSamples.clear();
  //           }),

  //       // Allow modules to orient and stabilize
  //       Commands.run(() -> drive.runCharacterization(0.0), drive)
  //           .withTimeout(Constants.DriveConstants.SLIP_START_DELAY),

  //       // Start timer
  //       Commands.runOnce(timer::restart),

  //       // Ramp voltage and gather data
  //       Commands.run(
  //               () -> {
  //                 double voltage = timer.get() * Constants.DriveConstants.SLIP_RAMP_RATE;
  //                 if (voltage > Constants.DriveConstants.SLIP_MAX_VOLTAGE) {
  //                   voltage = Constants.DriveConstants.SLIP_MAX_VOLTAGE;
  //                 }

  //                 drive.runCharacterization(voltage);

  //                 // Collect data from all modules
  //                 double[] currents = drive.getSlipCharacterizationCurrents();
  //                 double avgCurrent = 0.0;
  //                 for (double current : currents) {
  //                   avgCurrent += current / 4.0;
  //                 }

  //                 currentSamples.add(avgCurrent);
  //                 velocitySamples.add(drive.getFFCharacterizationVelocity());
  //                 voltageSamples.add(voltage);
  //               },
  //               drive)

  //           // When cancelled, calculate and print results
  //           .finallyDo(
  //               () -> {

  //                 // Analyze data for slip detection
  //                 double[] currents = drive.getSlipCharacterizationCurrents();
  //                 double slipCurrent = detectSlipCurrent(currentSamples, velocitySamples);

  //                 NumberFormat formatter = new DecimalFormat("#0.0");
  //                 System.out.println(
  //                     "\tDetected Slip Current: " + formatter.format(slipCurrent) + " A");
  //                 System.out.println("\tIndividual Module Currents:");
  //                 for (int i = 0; i < 4; i++) {
  //                   System.out.println(
  //                       "\t\tModule " + i + ": " + formatter.format(currents[i]) + " A");
  //                 }
  //               }));
  // }
  // // based on
  // //
  // https://github.com/Mechanical-Advantage/AdvantageKit/blob/main/docs/docs/getting-started/template-projects/talonfx-swerve-template.md#L116-L239
  // /**
  //  * Detects the slip current from collected data samples. Slip occurs when wheels first start
  //  * spinning significantly (velocity derivative increases).
  //  */
  // private static double detectSlipCurrent(
  //     List<Double> currentSamples, List<Double> velocitySamples) {

  //   // Thresholds for slip detection - using Constants from Constants.java
  //   final double VELOCITY_THRESHOLD =
  //       Constants.DriveConstants
  //           .SLIP_VELOCITY_THRESHOLD; // Velocity derivative indicating wheels started spinning
  //   final double MIN_CURRENT_THRESHOLD =
  //       Constants.DriveConstants.SLIP_MIN_CURRENT_THRESHOLD; // Minimum current just in case

  //   double maxCurrent = 0.0;
  //   int slipIndex = currentSamples.size() - 1;

  //   // Check all samples for slip detection
  //   for (int i = 1; i < currentSamples.size() - 1; i++) {
  //     // Calculate velocity derivative
  //     double velocityDerivative = velocitySamples.get(i + 1) - velocitySamples.get(i - 1);
  //     double currentValue = currentSamples.get(i);

  //     // This indicates wheels have overcome grip have begun to slip
  //     if (velocityDerivative > VELOCITY_THRESHOLD && currentValue > MIN_CURRENT_THRESHOLD) {
  //       slipIndex = i;
  //       break;
  //     }

  //     maxCurrent = Math.max(maxCurrent, currentValue);
  //   }

  //   return slipIndex < currentSamples.size() ? currentSamples.get(slipIndex) : maxCurrent;
  // }

  /** Measures the robot's wheel radius by spinning in a circle. */
  public static Command wheelRadiusCharacterization(DriveSubsystem drive) {
    SlewRateLimiter limiter = new SlewRateLimiter(Constants.DriveConstants.WHEEL_RADIUS_RAMP_RATE);
    WheelRadiusCharacterizationState state = new WheelRadiusCharacterizationState();
    Logger.recordOutput(
        "Commands/WheelRadiusCharacterization/DriveBaseRadius",
        Constants.DriveConstants.DRIVE_BASE_RADIUS);

    return Commands.parallel(
        // Drive control sequence
        Commands.sequence(
            // Reset acceleration limiter
            Commands.runOnce(
                () -> {
                  limiter.reset(0.0);
                }),

            // Turn in place, accelerating up to full speed
            Commands.run(
                () -> {
                  double speed =
                      limiter.calculate(Constants.DriveConstants.WHEEL_RADIUS_MAX_VELOCITY);
                  Logger.recordOutput("Commands/WheelRadiusCharacterization/Speed", speed);
                  drive.setControl(
                      new SwerveRequest.ApplyRobotSpeeds()
                          .withSpeeds(new ChassisSpeeds(0.0, 0.0, speed))
                          .withDriveRequestType(DriveRequestType.Velocity)
                          .withDesaturateWheelSpeeds(true));
                },
                drive)),

        // Measurement sequence
        Commands.sequence(
            // Wait for modules to fully orient before starting measurement
            Commands.waitSeconds(1.0),

            // Record starting measurement
            Commands.runOnce(
                () -> {
                  state.positions = drive.getWheelRadiusCharacterizationPositions();
                  state.lastAngle = drive.getRotation();
                  state.gyroDelta = 0.0;
                }),

            // Update gyro delta
            Commands.run(
                    () -> {
                      var rotation = drive.getRotation();
                      state.gyroDelta += Math.abs(rotation.minus(state.lastAngle).getRadians());
                      state.lastAngle = rotation;
                    })

                // When cancelled, calculate and print results
                .finallyDo(
                    () -> {
                      double[] positions = drive.getWheelRadiusCharacterizationPositions();
                      double wheelDelta = 0.0;
                      for (int i = 0; i < 4; i++) {
                        wheelDelta += Math.abs(positions[i] - state.positions[i]) / 4.0;
                      }
                      double wheelRadius =
                          (state.gyroDelta * Constants.DriveConstants.DRIVE_BASE_RADIUS)
                              / wheelDelta;

                      NumberFormat formatter = new DecimalFormat("#0.000");
                      System.out.println(
                          "********** Wheel Radius Characterization Results **********");
                      System.out.println(
                          "\tWheel Delta: " + formatter.format(wheelDelta) + " radians");
                      System.out.println(
                          "\tGyro Delta: " + formatter.format(state.gyroDelta) + " radians");
                      System.out.println(
                          "\tWheel Radius: "
                              + formatter.format(wheelRadius)
                              + " meters, "
                              + formatter.format(Units.metersToInches(wheelRadius))
                              + " inches");
                    })));
  }

  private static class WheelRadiusCharacterizationState {
    double[] positions = new double[4];
    Rotation2d lastAngle = Rotation2d.kZero;
    double gyroDelta = 0.0;
  }
}
