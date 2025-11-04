package frc.robot.auto;

import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.EndEffectorConstants.ClawState;
import frc.robot.Constants.IntakeConstants.IntakeState;
import frc.robot.Field.FieldUtils;
import frc.robot.RobotContainer;
import frc.robot.RobotState;
import frc.robot.RobotState.ScoreLevel;
import frc.robot.commands.DriveToCoralCommand;
import frc.robot.commands.GarageDriveToPoseAutoCommand;
import frc.robot.commands.Score;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.end_effector.Claw;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.superstructure.CoralStateTracker;
import frc.robot.subsystems.superstructure.CoralStateTracker.CoralPosition;
import frc.robot.subsystems.superstructure.Superstructure;
import frc.robot.subsystems.superstructure.SuperstructureState;
import frc.robot.subsystems.vision.Vision;
import frc.robot.util.MathHelpers;

public class NamedCommandsSetup {
  public NamedCommandsSetup(RobotContainer container, RobotState robotState) {
    Superstructure superstructure = container.getSuperStructure();
    Intake intake = container.getIntake();
    Claw claw = container.getClaw();
    Vision vision = container.getVision();
    DriveSubsystem drive = container.getDrive();

    // ====================AUTO STARTUP COMMANDS====================
    NamedCommands.registerCommand(
        "SuperStructureStartup",
        superstructure.setStateCommand(SuperstructureState.STOW, "STOW").asProxy());

    NamedCommands.registerCommand(
        "IntakeStartup", intake.setIntakeStateCommand(IntakeState.IDLE).asProxy());

    NamedCommands.registerCommand(
        "ScoreTargetStartup", // SET AS L4
        new InstantCommand(
            () -> robotState.getStoredScorePosition().setCoralScoreLevel(ScoreLevel.L4)));

    // ====================AUTO SCORING COMMANDS====================
    NamedCommands.registerCommand(
        "AimL4", superstructure.setStateCommand(SuperstructureState.L4_AIM, "L4 AIM").asProxy());

    NamedCommands.registerCommand(
        "AimL1", superstructure.setStateCommand(SuperstructureState.L1_AIM, "L1 AIM").asProxy());

    NamedCommands.registerCommand(
        "FinalLeftPoleAlign",
        new GarageDriveToPoseAutoCommand(
                container,
                () ->
                    MathHelpers.getPerpendicularOffsetPose(
                        FieldUtils.getClosestReef().leftPole.getPose(),
                        DriveConstants.AUTO_ALIGN_PERPENDICULAR_OFFSET))
            .withTimeout(3.0)); // Gets X seconds to perform action

    NamedCommands.registerCommand(
        "FinalRightPoleAlign",
        new GarageDriveToPoseAutoCommand(
                container,
                () ->
                    MathHelpers.getPerpendicularOffsetPose(
                        FieldUtils.getClosestReef().rightPole.getPose(),
                        DriveConstants.AUTO_ALIGN_PERPENDICULAR_OFFSET))
            .withTimeout(3.0));

    NamedCommands.registerCommand(
        "ConfirmScore", new Score(superstructure, claw, robotState, container));

    NamedCommands.registerCommand(
        "StowRobotState",
        new WaitUntilCommand(() -> robotState.isSafeToStow())
            .andThen(superstructure.setStateCommand(SuperstructureState.STOW, "STOW").asProxy()));

    // ====================AUTO INTAKE COMMANDS====================
    NamedCommands.registerCommand(
        "IntakeEnable",
        intake
            .setIntakeStateCommand(IntakeState.INTAKE)
            .asProxy()
            .alongWith(claw.setClawStateCommand(ClawState.INTAKING_CORAL).asProxy()));

    NamedCommands.registerCommand(
        "DriveToCoral",
        new DriveToCoralCommand(drive, vision)
            .withTimeout(10.0)); // Gets X seconds to perform action

    NamedCommands.registerCommand(
        "isSuperStructureAtL4",
        new WaitUntilCommand(
            () -> RobotState.getSuperstructureState() == SuperstructureState.L4_AIM));
    // ====================AUTO CORAL TRACKER COMMANDS====================
    NamedCommands.registerCommand(
        "SeesCoral",
        new WaitUntilCommand(() -> vision.getCoralPose().isPresent())
            .withTimeout(10.0)); // Gets X seconds to perform action

    NamedCommands.registerCommand(
        "IsCoralInFeeder", new WaitUntilCommand(() -> CoralStateTracker.hasCoralAboveIntake()));

    NamedCommands.registerCommand(
        "IsCoralInEndEffector",
        new WaitUntilCommand(
            () -> CoralStateTracker.getCurrentPosition() == CoralPosition.STAGED_IN_END_EFFECTOR));

    NamedCommands.registerCommand(
        "IsCoralInRobot", new WaitUntilCommand(() -> CoralStateTracker.hasCoral()));

    // ====================PID DRIVE COMMANDS====================
    // NamedCommands.registerCommand("DriveStraightRed", new DriveToPosePIDCommand(drive, () -> new
    // Pose2d(6.043, 4.060, Rotation2d.k180deg)));
    // NamedCommands.registerCommand("DriveStraightBlue", new DriveToPosePIDCommand(PEND, PEND,
    // PEND));
  }
}
