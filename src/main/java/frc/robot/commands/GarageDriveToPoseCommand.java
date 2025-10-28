package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest.ApplyRobotSpeeds;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Field.FieldConstants;
import frc.robot.Field.FieldUtils;
import frc.robot.RobotState;
import frc.robot.subsystems.drive.DriveSubsystem;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class GarageDriveToPoseCommand extends Command {
  private final ProfiledPIDController distanceController =
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

  private double ffMinRadius = 0.0, ffMaxRadius = 0.1;

  private final ApplyRobotSpeeds robotSpeeds =
      new ApplyRobotSpeeds()
          .withDriveRequestType(DriveRequestType.Velocity)
          .withDesaturateWheelSpeeds(true);

  public GarageDriveToPoseCommand(DriveSubsystem drive, Supplier<Pose2d> targetPoseSupplier) {
    addRequirements(drive);

    this.drive = drive;
    this.targetPoseSupplier = targetPoseSupplier;
    distanceController.setTolerance(Units.inchesToMeters(0.5));
    angleController.setTolerance(Units.degreesToRadians(1.5));
  }

  public GarageDriveToPoseCommand withJoystickRumble(Command rumbleCommand) {
    atSetpoint().onTrue(Commands.deferredProxy(() -> rumbleCommand));

    return this;
  }

  // Heavily inspired by 6328's `AutoAlignController` from 2024
  // https://github.com/Mechanical-Advantage/RobotCode2024/blob/main/src/main/java/org/littletonrobotics/frc2024/subsystems/drive/controllers/AutoAlignController.java#L135
  @Override
  public void execute() {

    Pose2d robotPoseOriginal = RobotState.getGlobalPose();
    Logger.recordOutput("Align Offset Inches", Constants.kAlignOffset);
    Logger.recordOutput("Align OffsetFB Inches", Constants.kAlignOffsetFB);
    Pose2d robot =
        robotPoseOriginal.plus(FieldConstants.getReefFaceOffset(FieldUtils.getClosestReef()));
    Pose2d target = targetPoseSupplier.get();

    double currentDistance = robot.getTranslation().getDistance(target.getTranslation());
    double ffScaler =
        MathUtil.clamp((currentDistance - ffMinRadius) / (ffMaxRadius - ffMinRadius), 0.0, 1.0);

    Translation2d translationalError = robot.getTranslation().minus(target.getTranslation());
    Rotation2d angularError = robot.getRotation().minus(target.getRotation());

    // Magnitude of translational velocity, meaning that x & y are controlled together
    double translationalSpeed =
        distanceController.getSetpoint().velocity * ffScaler
            + MathUtil.clamp(
                distanceController.calculate(translationalError.getNorm(), 0),
                -DriveConstants.kDriveMaxSpeed / 3,
                DriveConstants.kDriveMaxSpeed / 3);
    translationalSpeed = !distanceController.atGoal() ? translationalSpeed : 0;

    double anglularVelocity =
        angleController.getSetpoint().velocity * ffScaler
            + angleController.calculate(angularError.getRadians(), 0);
    anglularVelocity = !angleController.atGoal() ? anglularVelocity : 0;

    Translation2d velocity = new Translation2d(translationalSpeed, translationalError.getAngle());

    ChassisSpeeds speeds =
        ChassisSpeeds.fromFieldRelativeSpeeds(
            velocity.getX(), velocity.getY(), anglularVelocity, robot.getRotation());

    drive.setControl(robotSpeeds.withSpeeds(speeds));

    Logger.recordOutput("Commands/" + getName() + "/Robot", robot);
    Logger.recordOutput("Commands/" + getName() + "/Target", target);
    Logger.recordOutput("Commands/" + getName() + "/Error/Translational", translationalError);
    Logger.recordOutput("Commands/" + getName() + "/Error/Angular", angularError);
    Logger.recordOutput(
        "Commands/" + getName() + "/AtGoal/Translation", distanceController.atGoal());
    Logger.recordOutput("Commands/" + getName() + "/AtGoal/Angle", angleController.atGoal());
    Logger.recordOutput("Commands/" + getName() + "/Speeds/Translational", translationalSpeed);
    Logger.recordOutput("Commands/" + getName() + "/Speeds/ChassisSpeeds", speeds);
  }

  @Override
  public void end(boolean interrupt) {
    drive.setControl(robotSpeeds.withSpeeds(new ChassisSpeeds()));
    distanceController.reset(0);
    angleController.reset(0);
  }

  public Trigger atSetpoint() {
    return new Trigger(() -> distanceController.atSetpoint() && angleController.atSetpoint());
  }

  // public Trigger canShoot(
  //     double distanceTolerance, double rotationTolerance, double velocityTolerance) {
  //   return new Trigger(
  //       () ->
  //           Math.abs(distanceController.getPositionError()) < distanceTolerance
  //               && Math.abs(angleController.getPositionError()) < rotationTolerance
  //               && Math.hypot(
  //                       drive.getChassisSpeeds().vxMetersPerSecond,
  //                       drive.getChassisSpeeds().vyMetersPerSecond)
  //                   < velocityTolerance);
  // }
}
