package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveRequest;
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
import frc.robot.RobotState;
import frc.robot.subsystems.drive.DriveSubsystem;
import java.util.function.Supplier;

public class DriveToPosePIDCommand extends Command {
  private final PIDController xController =
      new PIDController(
          DriveConstants.DRIVE_TO_POSE_KP,
          DriveConstants.DRIVE_TO_POSE_KI,
          DriveConstants.DRIVE_TO_POSE_KD);
  private final PIDController yController =
      new PIDController(
          DriveConstants.DRIVE_TO_POSE_KP,
          DriveConstants.DRIVE_TO_POSE_KI,
          DriveConstants.DRIVE_TO_POSE_KD);
  private final ProfiledPIDController angleController =
      new ProfiledPIDController(
          DriveConstants.ANGLE_KP,
          0.0,
          DriveConstants.ANGLE_KD,
          new TrapezoidProfile.Constraints(
              DriveConstants.kDriveMaxAngularRate, DriveConstants.ANGLE_MAX_ACCELERATION));
  private final Supplier<Pose2d> targetPoseSupplier;
  private final DriveSubsystem drive;

  private final SwerveRequest.ApplyRobotSpeeds robotSpeeds = new SwerveRequest.ApplyRobotSpeeds();

  public DriveToPosePIDCommand(DriveSubsystem drive, Supplier<Pose2d> targetPoseSupplier) {
    addRequirements(drive);
    this.drive = drive;
    this.targetPoseSupplier = targetPoseSupplier;
    xController.setTolerance(DriveConstants.AUTO_ALIGN_NORM_TOLERANCE);
    yController.setTolerance(DriveConstants.AUTO_ALIGN_NORM_TOLERANCE);
    angleController.enableContinuousInput(-Math.PI, Math.PI);
  }

  @Override
  public void initialize() {
    angleController.reset(RobotState.getGlobalPose().getRotation().getRadians());
  }

  @Override
  public void execute() {
    Pose2d currentPose = RobotState.getGlobalPose();
    Pose2d targetPose = targetPoseSupplier.get();

    double xError = currentPose.getX() - targetPose.getX();
    double yError = currentPose.getY() - targetPose.getY();

    double xSpeed = xController.calculate(FieldUtils.getFlipped() * xError, 0.0);
    double ySpeed = yController.calculate(FieldUtils.getFlipped() * yError, 0.0);
    double omega =
        angleController.calculate(
            currentPose.getRotation().getRadians(), targetPose.getRotation().getRadians());

    ChassisSpeeds speeds = new ChassisSpeeds(xSpeed, ySpeed, omega);

    drive.setControl(
        robotSpeeds.withSpeeds(
            ChassisSpeeds.fromFieldRelativeSpeeds(
                speeds,
                FieldUtils.isRedAlliance()
                    ? RobotState.getGlobalPose().getRotation().plus(Rotation2d.k180deg)
                    : RobotState.getGlobalPose().getRotation())));
  }

  @Override
  public void end(boolean interrupt) {
    xController.reset();
    yController.reset();
    angleController.reset(RobotState.getGlobalPose().getRotation().getRadians());
  }

  @Override
  public boolean isFinished() {
    return xController.atSetpoint() && yController.atSetpoint() && angleController.atSetpoint();
  }

  public Trigger atSetpoint() {
    return new Trigger(
        () -> xController.atSetpoint() && yController.atSetpoint() && angleController.atSetpoint());
  }
}
