package frc.robot.commands;

import static edu.wpi.first.units.Units.Centimeters;
import static edu.wpi.first.units.Units.Degrees;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest.FieldCentricFacingAngle;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.DriveConstants;
import frc.robot.RobotState;
import frc.robot.subsystems.drive.DriveSubsystem;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;
import org.therekrabs.autopilot.APConstraints;
import org.therekrabs.autopilot.APProfile;
import org.therekrabs.autopilot.APResult;
import org.therekrabs.autopilot.APTarget;
import org.therekrabs.autopilot.Autopilot;

/**
 * Command that drives the robot to a target pose using the 3414 AutoPilot library.
 *
 * <p>This command uses the AutoPilot library for smooth path following with configurable motion
 * constraints, error tolerances, and beeline radius.
 */
public class DriveToPoseAutopilotCommand extends Command {
  // Remove 'static final' to make them instance variables
  private APConstraints constraints;
  private APProfile profile;
  private final Autopilot autopilot;
  private final DriveSubsystem drive;
  private final Supplier<Pose2d> targetPoseSupplier;

  private final FieldCentricFacingAngle facingAngleRequest =
      new FieldCentricFacingAngle()
          .withDriveRequestType(DriveRequestType.Velocity)
          .withDesaturateWheelSpeeds(true)
          .withHeadingPID(
              DriveConstants.AUTOPILOT_HEADING_KP,
              DriveConstants.AUTOPILOT_HEADING_KI,
              DriveConstants.AUTOPILOT_HEADING_KD);

  /**
   * Creates a new DriveToPoseAutopilotCommand.
   *
   * @param drive The drive subsystem to control
   * @param targetPoseSupplier Supplier that provides the target pose to drive to
   */
  public DriveToPoseAutopilotCommand(DriveSubsystem drive, Supplier<Pose2d> targetPoseSupplier) {
    this.drive = drive;
    this.targetPoseSupplier = targetPoseSupplier;
    
    // Initialize with default values
    this.constraints = new APConstraints()
        .withAcceleration(DriveConstants.AUTOPILOT_MAX_ACCELERATION)
        .withJerk(DriveConstants.AUTOPILOT_MAX_JERK);
    
    this.profile = new APProfile(constraints)
        .withErrorXY(Centimeters.of(DriveConstants.AUTOPILOT_ERROR_XY_METERS * 100))
        .withErrorTheta(Degrees.of(DriveConstants.AUTOPILOT_ERROR_THETA_DEGREES))
        .withBeelineRadius(Centimeters.of(DriveConstants.AUTOPILOT_BEELINE_RADIUS_METERS * 100));
    
    this.autopilot = new Autopilot(profile);
    addRequirements(drive);
  }

  /**
   * Creates a new DriveToPoseAutopilotCommand with a fixed target pose.
   *
   * @param drive The drive subsystem to control
   * @param targetPose The target pose to drive to
   */
  public DriveToPoseAutopilotCommand(DriveSubsystem drive, Pose2d targetPose) {
    this(drive, () -> targetPose);
  }

  // Add methods to change profile/constraints on the fly
  public DriveToPoseAutopilotCommand withConstraints(double acceleration, double jerk) {
    this.constraints = new APConstraints()
        .withAcceleration(acceleration)
        .withJerk(jerk);
    // Note: You may need to recreate the Autopilot instance with new profile
    return this;
  }

  public DriveToPoseAutopilotCommand withTolerance(double xyMeters, double thetaDegrees) {
    this.profile = new APProfile(constraints)
        .withErrorXY(Centimeters.of(xyMeters * 100))
        .withErrorTheta(Degrees.of(thetaDegrees))
        .withBeelineRadius(Centimeters.of(DriveConstants.AUTOPILOT_BEELINE_RADIUS_METERS * 100));
    return this;
  }

  @Override
  public void initialize() {
    Logger.recordOutput("Commands/" + getName() + "/Active", true);
  }

  @Override
  public void execute() {
    // Get current robot state
    Pose2d currentPose = RobotState.getGlobalPose();
    ChassisSpeeds robotRelativeSpeeds = drive.getChassisSpeeds();

    // Get target pose
    Pose2d targetPose = targetPoseSupplier.get();

    // Create AutoPilot target
    APTarget target =
        new APTarget(
            targetPose.getX(),
            targetPose.getY(),
            targetPose.getRotation().getRadians(),
            0.0); // Target velocity = 0 (stop at target)

    // Calculate desired velocities using AutoPilot
    APResult result = autopilot.calculate(currentPose, robotRelativeSpeeds, target);

    // Apply the calculated velocities to the drive
    drive.setControl(
        facingAngleRequest
            .withVelocityX(result.vx())
            .withVelocityY(result.vy())
            .withTargetDirection(result.targetAngle()));

    // Log telemetry
    Logger.recordOutput("Commands/" + getName() + "/CurrentPose", currentPose);
    Logger.recordOutput("Commands/" + getName() + "/TargetPose", targetPose);
    Logger.recordOutput("Commands/" + getName() + "/VelocityX", result.vx());
    Logger.recordOutput("Commands/" + getName() + "/VelocityY", result.vy());
    Logger.recordOutput("Commands/" + getName() + "/TargetAngle", result.targetAngle());
    Logger.recordOutput("Commands/" + getName() + "/AtTarget", autopilot.atTarget(currentPose, target));
  }

  @Override
  public void end(boolean interrupted) {
    // Stop the robot
    drive.setControl(
        facingAngleRequest
            .withVelocityX(0.0)
            .withVelocityY(0.0)
            .withTargetDirection(RobotState.getGlobalPose().getRotation()));
    
    Logger.recordOutput("Commands/" + getName() + "/Active", false);
    Logger.recordOutput("Commands/" + getName() + "/Interrupted", interrupted);
  }

  @Override
  public boolean isFinished() {
    Pose2d currentPose = RobotState.getGlobalPose();
    Pose2d targetPose = targetPoseSupplier.get();
    APTarget target =
        new APTarget(
            targetPose.getX(),
            targetPose.getY(),
            targetPose.getRotation().getRadians(),
            0.0);
    
    return autopilot.atTarget(currentPose, target);
  }

  /**
   * Returns a trigger that is true when the robot is at the target pose.
   *
   * @return A trigger that activates when the robot reaches the target
   */
  public Trigger atTarget() {
    return new Trigger(() -> {
      Pose2d currentPose = RobotState.getGlobalPose();
      Pose2d targetPose = targetPoseSupplier.get();
      APTarget target =
          new APTarget(
              targetPose.getX(),
              targetPose.getY(),
              targetPose.getRotation().getRadians(),
              0.0);
      return autopilot.atTarget(currentPose, target);
    });
  }
}

