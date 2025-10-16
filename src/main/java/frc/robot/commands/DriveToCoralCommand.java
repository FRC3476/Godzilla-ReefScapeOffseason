package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.RobotState;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.superstructure.CoralStateTracker;
import frc.robot.subsystems.superstructure.CoralStateTracker.CoralPosition;
import frc.robot.subsystems.vision.Vision;
import org.littletonrobotics.junction.Logger;

public class DriveToCoralCommand extends Command {
  private final ProfiledPIDController angleController =
      new ProfiledPIDController(
          DriveConstants.ANGLE_KP,
          0.0,
          0.0,
          new TrapezoidProfile.Constraints(
              DriveConstants.kDriveMaxAngularRate, DriveConstants.ANGLE_MAX_ACCELERATION));

  private final DriveSubsystem drive;
  private final Vision vision;

  private final SwerveRequest.ApplyRobotSpeeds robotSpeeds = new SwerveRequest.ApplyRobotSpeeds();

  public DriveToCoralCommand(DriveSubsystem drive, Vision vision) {
    addRequirements(drive, vision);

    this.drive = drive;
    this.vision = vision;
    angleController.enableContinuousInput(-Math.PI, Math.PI);
    angleController.setTolerance(Units.degreesToRadians(1.5));
  }

  @Override
  public void initialize() {
    // arm center is the same as the robot center when stowed, so can use field to
    // robot

    angleController.reset(0);
  }

  @Override
  public void execute() {
    Pose2d robotPose = RobotState.getGlobalPose();
    double currentDegrees = Math.abs(robotPose.getRotation().getDegrees());
    double rot_ffScaler = MathUtil.clamp(currentDegrees / 10, 0.0, 10.0);

    Rotation2d desiredTheta =
        robotPose.getRotation().plus(Rotation2d.fromDegrees(vision.getCoralTx()));

    double tx = vision.getCoralTx();
    Logger.recordOutput("Commands/" + getName() + "/TX Error", tx);

    double ty = vision.getCoralTy();
    Logger.recordOutput("Commands/" + getName() + "/TY Error", ty);

    double perpendicularSpeed = ty * 0.1;

    if (Math.abs(tx) > 10) {
      perpendicularSpeed = 0;
    }
    Logger.recordOutput("Commands/" + getName() + "/perpendicularSpeed", perpendicularSpeed);

    double angularSpeed =
        angleController.getSetpoint().velocity * rot_ffScaler
            + angleController.calculate(tx, 0)
            + Math.copySign(1.5, angleController.calculate(tx, 0));
    angularSpeed = !angleController.atSetpoint() ? angularSpeed : 0;
    Logger.recordOutput("Commands/" + getName() + "/angularSpeed", angularSpeed);

    ChassisSpeeds speeds = new ChassisSpeeds(perpendicularSpeed, 0.0, angularSpeed);

    drive.setControl(robotSpeeds.withSpeeds(speeds));
  }

  @Override
  public void end(boolean interrupt) {
    angleController.reset(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return CoralStateTracker.getCurrentPosition() != CoralPosition.NONE;
  }
}
