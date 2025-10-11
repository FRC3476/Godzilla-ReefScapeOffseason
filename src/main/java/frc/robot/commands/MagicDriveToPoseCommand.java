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
import frc.robot.RobotState;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.util.PoseUtils;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class MagicDriveToPoseCommand extends Command {
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

  public MagicDriveToPoseCommand(DriveSubsystem drive, Supplier<Pose2d> targetPoseSupplier) {
    addRequirements(drive);

    this.drive = drive;
    this.targetPoseSupplier = targetPoseSupplier;
  }

  @Override
  public void execute() {
    Pose2d robotPose = RobotState.getGlobalPose();
    Pose2d originalPose2d = targetPoseSupplier.get();
    Pose2d targetPose =
        originalPose2d.rotateAround(originalPose2d.getTranslation(), Rotation2d.kPi);

    Logger.recordOutput("Commands/" + getName() + "/targetPose", targetPose);

    Rotation2d desiredTheta = targetPose.getRotation().plus(Rotation2d.kPi);

    double perpendicularError = PoseUtils.getPerpendicularError(robotPose, targetPose);
    Logger.recordOutput("Commands/" + getName() + "/PerpendicularError", perpendicularError);

    double parallelError = PoseUtils.getParallelError(robotPose, targetPose);
    Logger.recordOutput("Commands/" + getName() + "/ParallelError", parallelError);

    double thetaError = robotPose.getRotation().minus(desiredTheta).getRadians();
    Logger.recordOutput("Commands/" + getName() + "/ThetaError", thetaError);

    double parallelSpeed = xController.calculate(-parallelError, 0);
    parallelSpeed = !xController.atSetpoint() ? parallelSpeed : 0;

    double perpendicularSpeed = yController.calculate(-perpendicularError, 0);
    if (thetaError < 0.05) {
      perpendicularSpeed = !yController.atSetpoint() ? perpendicularSpeed : 0;
    } else {
      perpendicularSpeed = 0;
    }

    double angularSpeed = angleController.calculate(thetaError, 0);
    angularSpeed = !angleController.atSetpoint() ? angularSpeed : 0;

    ChassisSpeeds speeds = new ChassisSpeeds(perpendicularSpeed, parallelSpeed, angularSpeed);

    drive.setControl(robotSpeeds.withSpeeds(speeds));
  }

  @Override
  public void end(boolean interrupt) {
    xController.reset();
    yController.reset();
    angleController.reset(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return xController.atSetpoint() && yController.atSetpoint() && angleController.atSetpoint();
  }

  public Trigger atSetpoint() {
    return new Trigger(
        () -> xController.atSetpoint() && yController.atSetpoint() && angleController.atSetpoint());
  }
}
