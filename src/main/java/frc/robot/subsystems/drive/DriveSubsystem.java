package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.*;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest.ApplyRobotSpeeds;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.Threads;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotState;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.vision.VisionFieldPoseEstimate;
import frc.robot.util.RobotTime;
import frc.robot.util.pathplanner.auto.AutoBuilder;
import frc.robot.util.pathplanner.config.ModuleConfig;
import frc.robot.util.pathplanner.config.PIDConstants;
import frc.robot.util.pathplanner.config.RobotConfig;
import frc.robot.util.pathplanner.controllers.PPHolonomicDriveController;
import frc.robot.util.pathplanner.controllers.PathFollowingController;
import frc.robot.util.pathplanner.trajectory.PathPlannerTrajectory;
import frc.robot.util.pathplanner.trajectory.PathPlannerTrajectoryState;
import frc.robot.util.pathplanner.util.PathPlannerLogging;
import frc.robot.util.simulations.MapleSimSwerveDrivetrain;
import java.util.function.Consumer;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

/**
 * The {@code DriveSubsystem} controls the robot's swerve drive base. It handles all driving
 * operations, including manual control, autonomous path following, and integration with sensors and
 * vision systems.
 */
public class DriveSubsystem extends SubsystemBase {
  DriveIO io;

  DriveIOInputsAutoLogged inputs = new DriveIOInputsAutoLogged();

  DriveViz telemetry = new DriveViz(Constants.DriveConstants.kDriveMaxSpeed);

  RobotState robotState;

  Controller controller;

  private final ApplyRobotSpeeds stopRequest =
      new ApplyRobotSpeeds().withDriveRequestType(DriveRequestType.OpenLoopVoltage);

  private final ApplyRobotSpeeds pathplannerAutoRequest =
      new ApplyRobotSpeeds()
          .withDriveRequestType(DriveRequestType.Velocity)
          .withDesaturateWheelSpeeds(true);

  public DriveSubsystem(DriveIO io, RobotState robotState) {
    this.io = io;
    this.robotState = robotState;

    configurePathPlanner();
  }

  private ChassisSpeeds applyDeadband(ChassisSpeeds input) {
    if (Math.hypot(input.vxMetersPerSecond, input.vyMetersPerSecond) < 0.05) {
      input.vxMetersPerSecond = input.vyMetersPerSecond = 0.0;
    }
    if (Math.abs(input.omegaRadiansPerSecond) < 0.05) {
      input.omegaRadiansPerSecond = 0.0;
    }
    return input;
  }

  private class Controller implements Consumer<PathPlannerTrajectory>, Runnable {
    private final PathFollowingController controller;
    private volatile PathPlannerTrajectory trajectory = null;
    private volatile Timer timer = null;
    private Notifier notifier;

    boolean hasSetPriority = false;

    public Controller(PathFollowingController controller) {
      this.controller = controller;
      this.timer = new Timer();
      notifier = new Notifier(this);
      notifier.startPeriodic(0.01);
    }

    @Override
    public void accept(PathPlannerTrajectory t) {
      trajectory = t;
      timer.reset();
      timer.start();
      controller.reset(null, null);
    }

    @Override
    public void run() {
      if (!hasSetPriority) {
        hasSetPriority = Threads.setCurrentThreadPriority(true, 41);
      }

      PathPlannerTrajectory traj = trajectory;

      if (traj == null) return;

      if (traj.isStayStoppedTrajectory()) {
        setControl(stopRequest);
        trajectory = null;
        return;
      }

      double now = timer.get();
      PathPlannerTrajectoryState targetState = traj.sample(now);

      ChassisSpeeds speeds =
          controller.calculateRobotRelativeSpeeds(
              robotState.getLatestFieldToRobot().getValue(), targetState);
      if (DriverStation.isEnabled()) {
        setControl(
            pathplannerAutoRequest
                .withSpeeds(applyDeadband(speeds))
                .withWheelForceFeedforwardsX(targetState.feedforwards.robotRelativeForcesXNewtons())
                .withWheelForceFeedforwardsY(
                    targetState.feedforwards.robotRelativeForcesYNewtons()));
      }
    }
  }

  @Override
  public void periodic() {
    double timestamp = RobotTime.getTimestampSeconds();
    io.readInputs(inputs);
    io.updateOperatorPerspective();
    telemetry.telemeterize(inputs);
    Logger.processInputs("DriveInputs", inputs);
    io.logModules(inputs);

    robotState.incrementIterationCount();
    if (DriverStation.isDisabled()) {
      configureStandardDevsForDisabled();
    } else {
      configureStandardDevsForEnabled();
    }
    Logger.recordOutput("Drive/latencyPeriodicSec", RobotTime.getTimestampSeconds() - timestamp);
    Logger.recordOutput(
        "Drive/currentCommand",
        (getCurrentCommand() == null) ? "Default" : getCurrentCommand().getName());
  }

  private void configurePathPlanner() {
    ModuleConfig moduleConfig =
        new ModuleConfig(
            Constants.DriveConstants.kDrivetrain.getModuleConstants()[0].WheelRadius,
            Constants.DriveConstants.kDriveMaxSpeed,
            Constants.DriveConstants.kWheelCoefficientOfFriction,
            DCMotor.getKrakenX60Foc(1),
            Constants.DriveConstants.kDrivetrain.getModuleConstants()[0].DriveMotorGearRatio,
            Constants.DriveConstants.kDrivetrain.getModuleConstants()[0].SlipCurrent,
            1);

    RobotConfig robotConfig =
        new RobotConfig(
            Constants.kRobotMassKg + Units.lbsToKilograms(0),
            Constants.kRobotMomentOfInertia,
            moduleConfig,
            Constants.kCOGHeightMeters,
            new Translation2d(0.31115, 0.31115),
            new Translation2d(0.31115, -0.31115),
            new Translation2d(-0.31115, 0.31115),
            new Translation2d(-0.31115, -0.31115));

    controller =
        new Controller(
            new PPHolonomicDriveController(
                new PIDConstants(Constants.AutoConstants.kPLTEController, 0.0, 0.0),
                new PIDConstants(Constants.AutoConstants.kPCTEController, 0.0, 0.0),
                new PIDConstants(Constants.AutoConstants.kPThetaController, 0.0, 0.0),
                0.01));

    AutoBuilder.configure(
        () -> RobotState.getGlobalPose(),
        (pose) -> {
          resetOdometry(pose);
        },
        () -> robotState.getLatestFusedRobotRelativeChassisSpeed(),
        controller,
        robotConfig,
        () -> robotState.isRedAlliance(),
        this);

    PathPlannerLogging.setLogTargetPoseCallback(
        (pose) -> {
          robotState.setTrajectoryTargetPose(pose);
          Logger.recordOutput("PathPlanner/targetPose", pose);
        });

    PathPlannerLogging.setLogCurrentPoseCallback(
        (pose) -> {
          robotState.setTrajectoryCurrentPose(pose);
          Logger.recordOutput("PathPlanner/currentPose", pose);
        });

    PathPlannerLogging.setLogActivePathCallback(
        (activePath) -> {
          Logger.recordOutput("PathPlanner/activePath", activePath.toArray(new Pose2d[0]));
        });

    PathPlannerLogging.setLogTargetChassisSpeedsCallback(
        (chassisSpeeds) -> {
          Logger.recordOutput("PathPlanner/targetChassisSpeeds", chassisSpeeds);
        });
  }

  public void resetOdometry(Pose2d pose) {
    io.resetOdometry(pose);
  }

  public Consumer<PathPlannerTrajectory> getController() {
    return controller;
  }

  public void setControl(SwerveRequest request) {
    io.setControl(request);
  }

  // API
  public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
    return io.applyRequest(requestSupplier, this).withName("Swerve drive request");
  }

  public void addVisionMeasurement(VisionFieldPoseEstimate visionFieldPoseEstimate) {
    io.addVisionMeasurement(visionFieldPoseEstimate);
  }

  public void setStateStdDevs(double xStd, double yStd, double rotStd) {
    io.setStateStdDevs(xStd, yStd, rotStd);
  }

  /** Returns the maximum linear speed in meters per sec. */
  public double getMaxLinearSpeedMetersPerSec() {
    return TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
  }

  /** Returns the maximum angular speed in radians per sec. */
  public double getMaxAngularSpeedRadPerSec() {
    return getMaxLinearSpeedMetersPerSec() / Constants.DriveConstants.DRIVE_BASE_RADIUS;
  }

  public void configureStandardDevsForDisabled() {
    setStateStdDevs(
        Constants.DriveConstants.kDisabledDriveXStdDev,
        Constants.DriveConstants.kDisabledDriveYStdDev,
        Constants.DriveConstants.kDisabledDriveRotStdDev);
  }

  public void configureStandardDevsForEnabled() {
    setStateStdDevs(
        Constants.DriveConstants.kEnabledDriveXStdDev,
        Constants.DriveConstants.kEnabledDriveYStdDev,
        Constants.DriveConstants.kEnabledDriveRotStdDev);
  }

  public MapleSimSwerveDrivetrain getMapleSimDrive() {
    if (io instanceof DriveIOSim) {
      return ((DriveIOSim) io).getMapleSimDrive();
    }
    return null;
  }
}
