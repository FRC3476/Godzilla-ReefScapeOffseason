package frc.robot.humanControls;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.IntakeConstants.IntakeState;
import frc.robot.RobotContainer;
import frc.robot.RobotState;
import frc.robot.RobotState.AlgaeIntake;
import frc.robot.RobotState.ScoreLevel;
import frc.robot.RobotState.ScorePosition;
import frc.robot.commands.DriveToCoralCommand;
import frc.robot.subsystems.climb.ClimbRoller;
import frc.robot.subsystems.climb.Climber;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.superstructure.Superstructure;
import frc.robot.subsystems.superstructure.SuperstructureState;
import frc.robot.subsystems.vision.Vision;
import frc.robot.util.COColor;
import frc.robot.util.Controls.StreamDeck.StreamDeck;
import frc.robot.util.Controls.StreamDeck.StreamDeck.CommandType;
import frc.robot.util.Controls.StreamDeck.StreamDeck.StreamDeckButtonType;
import frc.robot.util.Controls.StreamDeck.StreamDeck.StreamDeckCommand;
import frc.robot.util.Controls.StreamDeck.StreamDeckButton;
import frc.robot.util.Controls.StreamDeck.StreamDeckButtonConfig;

public class OperatorControls {
  private final RobotContainer container;
  private final Elevator elevator;
  private final DriveSubsystem drive;
  private final Superstructure superstructure;
  private final Intake intake;
  private final Climber climber;
  private final StreamDeck streamdeck;
  private final ClimbRoller climbRoller;
  private final RobotState robotState;
  private final Vision vision;

  public OperatorControls(RobotContainer container, StreamDeck streamdeck, RobotState robotState) {
    this.container = container;
    this.streamdeck = streamdeck;
    this.robotState = robotState;
    drive = container.getDrive();
    superstructure = container.getSuperStructure();
    elevator = container.getElevator();
    intake = container.getIntake();
    climber = container.getClimber();
    climbRoller = container.getClimbRoller();
    vision = container.getVision();
    configureDriveStreamDeckBindings();
  }

  private void configureDriveStreamDeckBindings() {
    StreamDeckButtonConfig orangeConfig =
        new StreamDeckButtonConfig(COColor.kCOOrangePure.toString(), COColor.kBlack.toString(), "");
    StreamDeckButtonConfig tealConfig =
        new StreamDeckButtonConfig(COColor.kCOTealPure.toString(), COColor.kWhite.toString(), "");
    StreamDeckButtonConfig tealOnWhiteConfig =
        new StreamDeckButtonConfig(COColor.kWhite.toString(), COColor.kCOTealPure.toString(), "");
    StreamDeckButtonConfig orangeOnWhiteConfig =
        new StreamDeckButtonConfig(COColor.kWhite.toString(), COColor.kCOOrangePure.toString(), "");
    StreamDeckButtonConfig redConfig =
        new StreamDeckButtonConfig(COColor.kRed.toString(), COColor.kBlack.toString(), "");
    StreamDeckButtonConfig yellowConfig =
        new StreamDeckButtonConfig(COColor.kYellow.toString(), COColor.kBlack.toString(), "");
    StreamDeckButtonConfig yellowOnBlackConfig =
        new StreamDeckButtonConfig(COColor.kBlack.toString(), COColor.kYellow.toString(), "");
    StreamDeckButtonConfig activeConfig =
        new StreamDeckButtonConfig(COColor.kGreen.toString(), COColor.kBlack.toString(), "");

    // --- Coral L4 ---
    streamdeck.addButton(
        StreamDeckButtonType.CUSTOM,
        new StreamDeckButton(0, 7, "Coral L4")
            .withInactiveConfig(orangeConfig)
            .withActiveConfig(activeConfig)
            .withText("L4"),
        () -> robotState.getStoredScorePosition().getCoralScoreLevel() == ScoreLevel.L4,
        new StreamDeckCommand(
            CommandType.ON_TRUE,
            Commands.runOnce(
                    () -> robotState.getStoredScorePosition().setCoralScoreLevel(ScoreLevel.L4))
                .ignoringDisable(true)));

    // --- Coral L3 ---
    streamdeck.addButton(
        StreamDeckButtonType.CUSTOM,
        new StreamDeckButton(1, 7, "Coral L3")
            .withInactiveConfig(orangeConfig)
            .withActiveConfig(activeConfig)
            .withText("L3"),
        () -> robotState.getStoredScorePosition().getCoralScoreLevel() == ScoreLevel.L3,
        new StreamDeckCommand(
            CommandType.ON_TRUE,
            Commands.runOnce(
                    () -> robotState.getStoredScorePosition().setCoralScoreLevel(ScoreLevel.L3))
                .ignoringDisable(true)));

    // --- Coral L2 ---
    streamdeck.addButton(
        StreamDeckButtonType.CUSTOM,
        new StreamDeckButton(2, 7, "Coral L2")
            .withInactiveConfig(orangeConfig)
            .withActiveConfig(activeConfig)
            .withText("L2"),
        () -> robotState.getStoredScorePosition().getCoralScoreLevel() == ScoreLevel.L2,
        new StreamDeckCommand(
            CommandType.ON_TRUE,
            Commands.runOnce(
                    () -> robotState.getStoredScorePosition().setCoralScoreLevel(ScoreLevel.L2))
                .ignoringDisable(true)));

    // --- Coral L1 ---
    streamdeck.addButton(
        StreamDeckButtonType.CUSTOM,
        new StreamDeckButton(3, 7, "Coral L1")
            .withInactiveConfig(orangeConfig)
            .withActiveConfig(activeConfig)
            .withText("L1"),
        () -> robotState.getStoredScorePosition().getCoralScoreLevel() == ScoreLevel.L1,
        new StreamDeckCommand(
            CommandType.ON_TRUE,
            Commands.runOnce(
                    () -> robotState.getStoredScorePosition().setCoralScoreLevel(ScoreLevel.L1))
                .ignoringDisable(true)));

    // --- Algae Barge ---
    streamdeck.addButton(
        StreamDeckButtonType.CUSTOM,
        new StreamDeckButton(0, 6, "Algae Barge")
            .withInactiveConfig(tealConfig)
            .withActiveConfig(activeConfig)
            .withText("B"),
        () -> robotState.getStoredScorePosition().getAlgaeScoreLevel() == ScoreLevel.BARGE,
        new StreamDeckCommand(
            CommandType.ON_TRUE,
            Commands.runOnce(
                    () -> robotState.getStoredScorePosition().setAlgaeScoreLevel(ScoreLevel.BARGE))
                .ignoringDisable(true)));

    // --- Algae L2 ---
    streamdeck.addButton(
        StreamDeckButtonType.CUSTOM,
        new StreamDeckButton(1, 6, "Algae L2")
            .withInactiveConfig(tealConfig)
            .withActiveConfig(activeConfig)
            .withText("L2"),
        () -> robotState.getStoredScorePosition().getAlgaeIntake() == AlgaeIntake.L2_ALGAE,
        new StreamDeckCommand(
            CommandType.ON_TRUE,
            Commands.runOnce(
                    () -> {
                      ScorePosition stored = robotState.getStoredScorePosition();
                      if (stored.getAlgaeIntake() == AlgaeIntake.L2_ALGAE) {
                        stored.setAlgaeIntake(AlgaeIntake.NONE);
                      } else {
                        stored.setAlgaeIntake(AlgaeIntake.L2_ALGAE);
                      }
                    })
                .ignoringDisable(true)));

    // --- Algae L1 ---
    streamdeck.addButton(
        StreamDeckButtonType.CUSTOM,
        new StreamDeckButton(2, 6, "Algae L1")
            .withInactiveConfig(tealConfig)
            .withActiveConfig(activeConfig)
            .withText("L1"),
        () -> robotState.getStoredScorePosition().getAlgaeIntake() == AlgaeIntake.L1_ALGAE,
        new StreamDeckCommand(
            CommandType.ON_TRUE,
            Commands.runOnce(
                    () -> {
                      ScorePosition stored = robotState.getStoredScorePosition();
                      if (stored.getAlgaeIntake() == AlgaeIntake.L1_ALGAE) {
                        stored.setAlgaeIntake(AlgaeIntake.NONE);
                      } else {
                        stored.setAlgaeIntake(AlgaeIntake.L1_ALGAE);
                      }
                    })
                .ignoringDisable(true)));

    // --- Algae Processor ---
    streamdeck.addButton(
        StreamDeckButtonType.CUSTOM,
        new StreamDeckButton(3, 6, "Algae Processor")
            .withInactiveConfig(tealConfig)
            .withActiveConfig(activeConfig)
            .withText("P"),
        () -> robotState.getStoredScorePosition().getAlgaeScoreLevel() == ScoreLevel.PROCESSOR,
        new StreamDeckCommand(
            CommandType.ON_TRUE,
            Commands.runOnce(
                    () ->
                        robotState
                            .getStoredScorePosition()
                            .setAlgaeScoreLevel(ScoreLevel.PROCESSOR))
                .ignoringDisable(true)));

    // --- Home Elevator ---
    Command homeElevatorButtonCommand =
        elevator.manualHomeElevator().withName("homeElevatorButton");
    streamdeck.addButton(
        StreamDeckButtonType.CUSTOM,
        new StreamDeckButton(2, 4, "Home Elevator")
            .withInactiveConfig(orangeOnWhiteConfig)
            .withActiveConfig(activeConfig)
            .withText("HE"),
        homeElevatorButtonCommand::isScheduled,
        new StreamDeckCommand(CommandType.ON_TRUE, homeElevatorButtonCommand));

    // --- Zero Gyro ---
    streamdeck.addButton(
        StreamDeckButtonType.PRESS,
        new StreamDeckButton(2, 3, "Zero Gyro 1")
            .withInactiveConfig(tealOnWhiteConfig)
            .withActiveConfig(activeConfig)
            .withText("GZ"));
    streamdeck.addButton(
        StreamDeckButtonType.PRESS,
        new StreamDeckButton(2, 5, "Zero Gyro 2")
            .withInactiveConfig(tealOnWhiteConfig)
            .withActiveConfig(activeConfig)
            .withText("GZ"));
    streamdeck
        .button("Zero Gyro 1")
        .and(streamdeck.button("Zero Gyro 2"))
        .onTrue(
            Commands.runOnce(
                    () ->
                        drive.resetOdometry(
                            new Pose2d(
                                RobotState.getGlobalPose().getTranslation(), Rotation2d.kZero)),
                    drive)
                .ignoringDisable(true));

    // --- Climbing ---
    Command climbDeployButtonCommand =
        Commands.runOnce(() -> climbRoller.setClimbing(true))
            .andThen(
                climbRoller
                    .holdCage()
                    .alongWith(intake.setIntakeStateCommand(IntakeState.IDLE).asProxy())
                    .alongWith(
                        superstructure
                            .setStateCommand(SuperstructureState.CLIMB, "Climb")
                            .asProxy()))
            .andThen(new WaitCommand(0.25))
            .andThen(climber.climbDeploy())
            .withName("climbDeployButton");
    Command climbClimbButtonCommand = climber.climbClimb().withName("climbClimbButton");
    Command climbManualButtonCommand = climber.climbOut(12).withName("manualClimbButton");
    Command manualClimbOffButtonCommand = climber.climbSTOP().withName("manualClimbButtonOff");
    StreamDeckButton climbDeployButton1 =
        new StreamDeckButton(1, 3, "Climb Deploy 1")
            .withInactiveConfig(yellowConfig)
            .withActiveConfig(activeConfig)
            .withText("C");
    StreamDeckButton climbDeployButton2 =
        new StreamDeckButton(1, 5, "Climb Deploy 2")
            .withInactiveConfig(yellowConfig)
            .withActiveConfig(activeConfig)
            .withText("C");
    StreamDeckButton climbClimbButton1 =
        new StreamDeckButton(0, 3, "Auto Climb 1")
            .withInactiveConfig(redConfig)
            .withActiveConfig(activeConfig)
            .withText("C");
    StreamDeckButton climbClimbButton2 =
        new StreamDeckButton(0, 5, "Auto Climb 2")
            .withInactiveConfig(redConfig)
            .withActiveConfig(activeConfig)
            .withText("C");
    StreamDeckButton climbManualButton =
        new StreamDeckButton(0, 4, "Manual Climb")
            .withInactiveConfig(yellowOnBlackConfig)
            .withActiveConfig(activeConfig)
            .withText("C");
    streamdeck.addButton(
        StreamDeckButtonType.CUSTOM,
        climbDeployButton1,
        climbDeployButtonCommand::isScheduled,
        new StreamDeckCommand(CommandType.ON_FALSE, manualClimbOffButtonCommand));
    streamdeck.addButton(
        StreamDeckButtonType.CUSTOM,
        climbDeployButton2,
        climbDeployButtonCommand::isScheduled,
        new StreamDeckCommand(CommandType.ON_FALSE, manualClimbOffButtonCommand));
    streamdeck.addButton(
        StreamDeckButtonType.CUSTOM,
        climbClimbButton1,
        climbClimbButtonCommand::isScheduled,
        new StreamDeckCommand(CommandType.ON_FALSE, manualClimbOffButtonCommand));
    streamdeck.addButton(
        StreamDeckButtonType.CUSTOM,
        climbClimbButton2,
        climbClimbButtonCommand::isScheduled,
        new StreamDeckCommand(CommandType.ON_FALSE, manualClimbOffButtonCommand));
    streamdeck.addButton(
        StreamDeckButtonType.CUSTOM, climbManualButton, climbManualButtonCommand::isScheduled);
    streamdeck
        .button(climbDeployButton1)
        .and(streamdeck.button(climbDeployButton2))
        .onTrue(climbDeployButtonCommand);
    streamdeck
        .button(climbClimbButton1)
        .and(streamdeck.button(climbClimbButton2))
        .and(streamdeck.button(climbManualButton).negate())
        .onTrue(climbClimbButtonCommand);
    streamdeck
        .button(climbClimbButton1)
        .and(streamdeck.button(climbClimbButton2))
        .and(streamdeck.button(climbManualButton))
        .onTrue(climbManualButtonCommand);

    // Auto Climb with streamdeck confirmation
    Trigger autoClimbTrigger = new Trigger(() -> climbRoller.hasCage()).debounce(.25);
    streamdeck
        .button(climbDeployButton1)
        .and(streamdeck.button(climbDeployButton2))
        .and(autoClimbTrigger)
        .onTrue(climber.climbClimb().withName("AutoClimb"));

    // --- Climb Roller Stop ---
    Command climbRollerStopButtonCommand =
        climbRoller.rollerSTOP().withName("climbRollerStopButton");
    streamdeck.addButton(
        StreamDeckButtonType.CUSTOM,
        new StreamDeckButton(1, 4, "Climb Roller Stop")
            .withInactiveConfig(redConfig)
            .withActiveConfig(activeConfig)
            .withText("CR0"),
        climbRollerStopButtonCommand::isScheduled,
        new StreamDeckCommand(CommandType.ON_TRUE, climbRollerStopButtonCommand));

    // --- Auto Score Left ---
    streamdeck.addButton(
        StreamDeckButtonType.PRESS,
        new StreamDeckButton(1, 0, "Left Plus 1")
            .withInactiveConfig(orangeConfig)
            .withActiveConfig(activeConfig)
            .withText("L+1"),
        new StreamDeckCommand(
            CommandType.ON_TRUE,
            // left and right are swapped on purpose to match the scoring POV
            Commands.runOnce(() -> robotState.offsetRight())));

    // --- Auto Score Right ---
    streamdeck.addButton(
        StreamDeckButtonType.PRESS,
        new StreamDeckButton(1, 2, "Right Plus 1")
            .withInactiveConfig(orangeConfig)
            .withActiveConfig(activeConfig)
            .withText("R+1"),
        new StreamDeckCommand(
            CommandType.ON_TRUE,
            // left and right are swapped on purpose to match the scoring POV
            Commands.runOnce(() -> robotState.offsetLeft())));

    // --- Auto Score Forward ---
    streamdeck.addButton(
        StreamDeckButtonType.PRESS,
        new StreamDeckButton(0, 1, "Forward Plus 1")
            .withInactiveConfig(orangeConfig)
            .withActiveConfig(activeConfig)
            .withText("F+1"),
        new StreamDeckCommand(
            CommandType.ON_TRUE, Commands.runOnce(() -> robotState.offsetForward())));

    // --- Auto Score Backward ---
    streamdeck.addButton(
        StreamDeckButtonType.PRESS,
        new StreamDeckButton(1, 1, "Backward Plus 1")
            .withInactiveConfig(orangeConfig)
            .withActiveConfig(activeConfig)
            .withText("B+1"),
        new StreamDeckCommand(
            CommandType.ON_TRUE, Commands.runOnce(() -> robotState.offsetBackward())));

    // --- Auto Score Zero ---
    streamdeck.addButton(
        StreamDeckButtonType.PRESS,
        new StreamDeckButton(2, 1, "Autoscore Zero")
            .withInactiveConfig(orangeConfig)
            .withActiveConfig(activeConfig)
            .withText("AS0"),
        new StreamDeckCommand(
            CommandType.ON_TRUE, Commands.runOnce(() -> robotState.offsetZero())));

    // --- Superstructure Manual Override ---
    streamdeck.addButton(
        StreamDeckButtonType.CUSTOM,
        new StreamDeckButton(3, 0, "Manual Override")
            .withInactiveBackground(COColor.kBlack.toString())
            .withInactiveForeground(COColor.kWhite.toString())
            .withActiveBackground(COColor.kRed.toString())
            .withActiveForeground(COColor.kYellow.toString())
            .withText("MO"),
        () -> RobotState.getSuperstructureManualOverrideMode(),
        new StreamDeckCommand(
            CommandType.ON_TRUE,
            Commands.runOnce(() -> RobotState.toggleSuperstructureManualOverrideMode())));

    // --- Drive To Coral ---
    Command driveToCoralButtonCommand = new DriveToCoralCommand(drive, vision);
    streamdeck.addButton(
        StreamDeckButtonType.CUSTOM,
        new StreamDeckButton(3, 1, "Drive to Coral")
            .withInactiveConfig(tealConfig)
            .withActiveConfig(activeConfig)
            .withText("DTC"),
        driveToCoralButtonCommand::isScheduled,
        new StreamDeckCommand(CommandType.WHILE_TRUE, driveToCoralButtonCommand));
  }
}
