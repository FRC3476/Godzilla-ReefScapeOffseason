package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest.ApplyRobotSpeeds;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.RobotState;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.util.MathHelpers;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class GarageDrivePerpendicularToPoseCommand extends Command {
  private final ProfiledPIDController parallelController =
      new ProfiledPIDController(
          DriveConstants.DRIVE_TO_POSE_KP,
          DriveConstants.DRIVE_TO_POSE_KI,
          DriveConstants.DRIVE_TO_POSE_KD,
          new TrapezoidProfile.Constraints(
              (Constants.DriveConstants.kDriveMaxSpeed / 3),
              Constants.DriveConstants.kMaxAccelerationMetersPerSecondSquared / 3));

  private final ProfiledPIDController angleController =
      new ProfiledPIDController(
          DriveConstants.ANGLE_KP,
          0.0,
          DriveConstants.ANGLE_KD,
          new TrapezoidProfile.Constraints(
              DriveConstants.kDriveMaxAngularRate, DriveConstants.ANGLE_MAX_ACCELERATION / 3));

  private final DriveSubsystem drive;
  private final Supplier<Pose2d> targetPoseSupplier;
  private final DoubleSupplier perpendicularInput;

  private double perpendicularError = 0;

  private final ApplyRobotSpeeds robotSpeeds =
      new ApplyRobotSpeeds()
          .withDriveRequestType(DriveRequestType.Velocity)
          .withDesaturateWheelSpeeds(true);

  public GarageDrivePerpendicularToPoseCommand(
      DriveSubsystem drive,
      Supplier<Pose2d> targetPoseSupplier,
      DoubleSupplier perpendicularInput) {
    addRequirements(drive);

    this.drive = drive;
    this.targetPoseSupplier = targetPoseSupplier;
    this.perpendicularInput = perpendicularInput;
  }

  public GarageDrivePerpendicularToPoseCommand withJoystickRumble(
      DoubleSupplier rumbleDistance, Command rumbleCommand) {
    atSetpoint(rumbleDistance).onTrue(Commands.deferredProxy(() -> rumbleCommand));

    return this;
  }

  @Override
  public void execute() {
    Pose2d robotPose = RobotState.getGlobalPose();
    Pose2d targetPose = targetPoseSupplier.get();
    Logger.recordOutput("Commands/" + getName() + "/targetPose", targetPose);

    Rotation2d desiredTheta = targetPose.getRotation().plus(Rotation2d.kPi);

    perpendicularError = MathHelpers.getPerpendicularError(robotPose, targetPose);
    Logger.recordOutput("Commands/" + getName() + "/PerpendicularError", perpendicularError);

    double parallelError = MathHelpers.getParallelError(robotPose, targetPose);
    Logger.recordOutput("Commands/" + getName() + "/ParallelError", parallelError);

    double thetaError = robotPose.getRotation().minus(desiredTheta).getRadians();
    Logger.recordOutput("Commands/" + getName() + "/ThetaError", thetaError);

    double parallelSpeed = parallelController.calculate(-parallelError, 0);
    parallelSpeed = !parallelController.atSetpoint() ? parallelSpeed : 0;

    double angularSpeed = angleController.calculate(thetaError, 0);
    angularSpeed = !angleController.atSetpoint() ? angularSpeed : 0;

    ChassisSpeeds speeds =
        new ChassisSpeeds(
            perpendicularInput.getAsDouble() * drive.getMaxLinearSpeedMetersPerSec(),
            parallelSpeed,
            angularSpeed);

    drive.setControl(robotSpeeds.withSpeeds(speeds));

    Logger.recordOutput(
        "Commands/" + getName() + "/ParallelAtSetpoint", parallelController.atSetpoint());
    Logger.recordOutput("Commands/" + getName() + "/AngleAtSetpoint", angleController.atSetpoint());
  }

  @Override
  public void end(boolean interrupt) {
    parallelController.reset(0);
    angleController.reset(0);
  }

  public Trigger atSetpoint(DoubleSupplier distance) {
    return new Trigger(
        () ->
            parallelController.atSetpoint()
                && angleController.atSetpoint()
                && Math.abs(perpendicularError) < distance.getAsDouble());
  }
}
