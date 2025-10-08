package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.DriveConstants;
import frc.robot.Field.FieldUtils;
import frc.robot.subsystems.drive.Drive;
import java.util.function.Supplier;

public class DriveToPosePIDCommand extends Command {
  private final PIDController driveController =
      new PIDController(
          DriveConstants.DRIVE_TO_POSE_KP,
          DriveConstants.DRIVE_TO_POSE_KI,
          DriveConstants.DRIVE_TO_POSE_KD);
  ProfiledPIDController angleController =
      new ProfiledPIDController(
          DriveConstants.ANGLE_KP,
          0.0,
          DriveConstants.ANGLE_KD,
          new TrapezoidProfile.Constraints(
              DriveConstants.ANGLE_MAX_VELOCITY, DriveConstants.ANGLE_MAX_ACCELERATION));
  private final Supplier<Pose2d> targetPoseSupplier;
  private final Drive drive;
  private boolean xAtSetpoint = false;
  private boolean yAtSetpoint = false;

  public DriveToPosePIDCommand(Drive drive, Supplier<Pose2d> targetPoseSupplier) {
    addRequirements(drive);
    this.drive = drive;
    this.targetPoseSupplier = targetPoseSupplier;
    angleController.enableContinuousInput(-Math.PI, Math.PI);
    angleController.reset(drive.getRotation().getRadians());
    driveController.setTolerance(DriveConstants.AUTO_ALIGN_NORM_TOLERANCE);
  }

  @Override
  public void execute() {
    Pose2d currentPose = drive.getPose();
    Pose2d targetPose = targetPoseSupplier.get();

    double xError = currentPose.getX() - targetPose.getX();
    double yError = currentPose.getY() - targetPose.getY();

    double xSpeed = driveController.calculate(FieldUtils.getFlipped() * xError, 0.0);
    xAtSetpoint = driveController.atSetpoint();
    double ySpeed = driveController.calculate(FieldUtils.getFlipped() * yError, 0.0);
    yAtSetpoint = driveController.atSetpoint();
    double omega =
        angleController.calculate(
            currentPose.getRotation().getRadians(), targetPose.getRotation().getRadians());

    ChassisSpeeds speeds = new ChassisSpeeds(xSpeed, ySpeed, omega);

    drive.runVelocity(
        ChassisSpeeds.fromFieldRelativeSpeeds(
            speeds,
            FieldUtils.isRedAlliance()
                ? drive.getRotation().plus(Rotation2d.k180deg)
                : drive.getRotation()));
  }

  @Override
  public void end(boolean interrupt) {
    driveController.reset();
    angleController.reset(drive.getRotation().getRadians());
  }

  @Override
  public boolean isFinished() {
    return xAtSetpoint && yAtSetpoint && angleController.atSetpoint();
  }

  public Trigger atSetpoint() {
    return new Trigger(() -> xAtSetpoint && yAtSetpoint && angleController.atSetpoint());
  }
}
