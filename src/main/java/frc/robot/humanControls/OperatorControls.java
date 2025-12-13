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
import frc.robot.subsystems.intake.IntakeOld;
import frc.robot.subsystems.led.LedState;
import frc.robot.subsystems.superstructure.Superstructure;
import frc.robot.subsystems.superstructure.SuperstructureState;
import frc.robot.subsystems.vision.Vision;
import frc.robot.util.Controls.StreamDeck.StreamDeck;
import frc.robot.util.Controls.StreamDeck.StreamDeckButton;
import frc.robot.util.Controls.StreamDeck.StreamDeckButtonConfig;
import java.util.HashMap;
import java.util.Map;
import java.util.Set;
import java.util.function.BooleanSupplier;

public class OperatorControls {
  private final RobotContainer container;
  private final Elevator elevator;
  private final DriveSubsystem drive;
  private final Superstructure superstructure;
  private final IntakeOld intake;
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
        new StreamDeckButtonConfig(LedState.kCOOrangePure.toString(), LedState.kOff.toString(), "");
    StreamDeckButtonConfig tealConfig =
        new StreamDeckButtonConfig(LedState.kCOTealPure.toString(), LedState.kWhite.toString(), "");
    StreamDeckButtonConfig tealOnWhiteConfig =
        new StreamDeckButtonConfig(LedState.kWhite.toString(), LedState.kCOTealPure.toString(), "");
    StreamDeckButtonConfig orangeOnWhiteConfig =
        new StreamDeckButtonConfig(
            LedState.kWhite.toString(), LedState.kCOOrangePure.toString(), "");
    StreamDeckButtonConfig redConfig =
        new StreamDeckButtonConfig(LedState.kRed.toString(), LedState.kOff.toString(), "");
    StreamDeckButtonConfig yellowConfig =
        new StreamDeckButtonConfig(LedState.kYellow.toString(), LedState.kOff.toString(), "");
    StreamDeckButtonConfig yellowOnBlackConfig =
        new StreamDeckButtonConfig(LedState.kOff.toString(), LedState.kYellow.toString(), "");
    StreamDeckButtonConfig activeConfig =
        new StreamDeckButtonConfig(LedState.kGreen.toString(), LedState.kOff.toString(), "");

    StreamDeckButton coralL4Button =
        new StreamDeckButton(0, 7, "Coral L4")
            .withInactiveConfig(orangeConfig)
            .withActiveConfig(activeConfig)
            .withText("L4");
    StreamDeckButton coralL3Button =
        new StreamDeckButton(1, 7, "Coral L3")
            .withInactiveConfig(orangeConfig)
            .withActiveConfig(activeConfig)
            .withText("L3");
    StreamDeckButton coralL2Button =
        new StreamDeckButton(2, 7, "Coral L2")
            .withInactiveConfig(orangeConfig)
            .withActiveConfig(activeConfig)
            .withText("L2");
    StreamDeckButton coralL1Button =
        new StreamDeckButton(3, 7, "Coral L1")
            .withInactiveConfig(orangeConfig)
            .withActiveConfig(activeConfig)
            .withText("L1");
    StreamDeckButton AlgaeBargeButton =
        new StreamDeckButton(0, 6, "Algae Barge")
            .withInactiveConfig(tealConfig)
            .withActiveConfig(activeConfig)
            .withText("B");
    StreamDeckButton AlgaeL2Button =
        new StreamDeckButton(1, 6, "Algae L2")
            .withInactiveConfig(tealConfig)
            .withActiveConfig(activeConfig)
            .withText("L2");
    StreamDeckButton AlgaeL1Button =
        new StreamDeckButton(2, 6, "Algae L1")
            .withInactiveConfig(tealConfig)
            .withActiveConfig(activeConfig)
            .withText("L1");
    // the Processor is handled by ronny, seperate button
    StreamDeckButton AlgaeProcessorButton =
        new StreamDeckButton(3, 6, "Algae Processor")
            .withInactiveConfig(tealConfig)
            .withActiveConfig(activeConfig)
            .withText("P");
    // StreamDeckButton ReefASideButton =
    //     new StreamDeckButton(3, 1, "Reef A Side")
    //         .withInactiveConfig(tealConfig)
    //         .withActiveConfig(activeConfig)
    //         .withText("A");
    // StreamDeckButton ReefBSideButton =
    //     new StreamDeckButton(2, 2, "Reef B Side")
    //         .withInactiveConfig(tealConfig)
    //         .withActiveConfig(activeConfig)
    //         .withText("B");
    // StreamDeckButton ReefCSideButton =
    //     new StreamDeckButton(1, 2, "Reef C Side")
    //         .withInactiveConfig(tealConfig)
    //         .withActiveConfig(activeConfig)
    //         .withText("C");
    // StreamDeckButton ReefDSideButton =
    //     new StreamDeckButton(0, 1, "Reef D Side")
    //         .withInactiveConfig(tealConfig)
    //         .withActiveConfig(activeConfig)
    //         .withText("D");
    // StreamDeckButton ReefESideButton =
    //     new StreamDeckButton(1, 0, "Reef E Side")
    //         .withInactiveConfig(tealConfig)
    //         .withActiveConfig(activeConfig)
    //         .withText("E");
    // StreamDeckButton ReefFSideButton =
    //     new StreamDeckButton(2, 0, "Reef F Side")
    //         .withInactiveConfig(tealConfig)
    //         .withActiveConfig(activeConfig)
    //         .withText("F");
    // StreamDeckButton reefRightSideButton =
    //     new StreamDeckButton(3, 4, "Reef Right Side 1")
    //         .withInactiveConfig(orangeConfig)
    //         .withActiveConfig(activeConfig)
    //         .withText("R");
    // StreamDeckButton reefRightSideButton2 =
    //     new StreamDeckButton(3, 5, "Reef Right Side 2")
    //         .withInactiveConfig(orangeConfig)
    //         .withActiveConfig(activeConfig)
    //         .withText("R");
    // StreamDeckButton reefLeftSideButton =
    //     new StreamDeckButton(3, 2, "Reef Left Side 1")
    //         .withInactiveConfig(orangeConfig)
    //         .withActiveConfig(activeConfig)
    //         .withText("L");
    // StreamDeckButton reefLeftSideButton2 =
    //     new StreamDeckButton(3, 3, "Reef Left Side 2")
    //         .withInactiveConfig(orangeConfig)
    //         .withActiveConfig(activeConfig)
    //         .withText("L");
    StreamDeckButton homeElevatorButton =
        new StreamDeckButton(2, 4, "Home Elevator")
            .withInactiveConfig(orangeOnWhiteConfig)
            .withActiveConfig(activeConfig)
            .withText("HE");
    StreamDeckButton zeroGyroButton =
        new StreamDeckButton(2, 3, "Zero Gyro 1")
            .withInactiveConfig(tealOnWhiteConfig)
            .withActiveConfig(activeConfig)
            .withText("GZ");
    StreamDeckButton zeroGyroButton2 =
        new StreamDeckButton(2, 5, "Zero Gyro 2")
            .withInactiveConfig(tealOnWhiteConfig)
            .withActiveConfig(activeConfig)
            .withText("GZ");
    StreamDeckButton climbDeployButton =
        new StreamDeckButton(1, 3, "Climb Deploy 1")
            .withInactiveConfig(yellowConfig)
            .withActiveConfig(activeConfig)
            .withText("C");
    StreamDeckButton climbRollerStopButton =
        new StreamDeckButton(1, 4, "Climb Roller Stop")
            .withInactiveConfig(redConfig)
            .withActiveConfig(activeConfig)
            .withText("CR0");
    StreamDeckButton climbDeployButton2 =
        new StreamDeckButton(1, 5, "Climb Deploy 2")
            .withInactiveConfig(yellowConfig)
            .withActiveConfig(activeConfig)
            .withText("C");
    StreamDeckButton climbClimbButton =
        new StreamDeckButton(0, 3, "Auto Climb 1")
            .withInactiveConfig(redConfig)
            .withActiveConfig(activeConfig)
            .withText("C");
    StreamDeckButton climbClimbButton2 =
        new StreamDeckButton(0, 5, "Auto Climb 2")
            .withInactiveConfig(redConfig)
            .withActiveConfig(activeConfig)
            .withText("C");
    StreamDeckButton manualClimbButton =
        new StreamDeckButton(0, 4, "Manual Climb")
            .withInactiveConfig(yellowOnBlackConfig)
            .withActiveConfig(activeConfig)
            .withText("C");
    // StreamDeckButton setManualScoringButton =
    //     new StreamDeckButton(0, 0, "Manual Score")
    //         .withInactiveConfig(orangeConfig)
    //         .withActiveConfig(activeConfig)
    //         .withText("MS");
    // StreamDeckButton setAutoScoringButton =
    //     new StreamDeckButton(0, 2, "Auto Score")
    //         .withInactiveConfig(orangeConfig)
    //         .withActiveConfig(activeConfig)
    //         .withText("AS");
    StreamDeckButton autoScoreLeftButton =
        new StreamDeckButton(1, 0, "Left Plus 1")
            .withInactiveConfig(orangeConfig)
            .withActiveConfig(activeConfig)
            .withText("L+1");
    StreamDeckButton autoScoreRightButton =
        new StreamDeckButton(1, 2, "Right Plus 1")
            .withInactiveConfig(orangeConfig)
            .withActiveConfig(activeConfig)
            .withText("R+1");
    StreamDeckButton autoScoreForwardButton =
        new StreamDeckButton(0, 1, "Forward Plus 1")
            .withInactiveConfig(orangeConfig)
            .withActiveConfig(activeConfig)
            .withText("F+1");
    StreamDeckButton autoScoreBackButton =
        new StreamDeckButton(1, 1, "Backward Plus 1")
            .withInactiveConfig(orangeConfig)
            .withActiveConfig(activeConfig)
            .withText("B+1");
    StreamDeckButton autoScoreZeroButton =
        new StreamDeckButton(2, 1, "Autoscore Zero")
            .withInactiveConfig(orangeConfig)
            .withActiveConfig(activeConfig)
            .withText("AS0");
    StreamDeckButton driveToCoralButton =
        new StreamDeckButton(3, 1, "Drive to Coral")
            .withInactiveConfig(tealConfig)
            .withActiveConfig(activeConfig)
            .withText("DTC");

    StreamDeckButton manualOverrideButton =
        new StreamDeckButton(3, 0, "Manual Override")
            .withInactiveBackground(LedState.kOff.toString())
            .withInactiveForeground(LedState.kWhite.toString())
            .withActiveBackground(LedState.kRed.toString())
            .withActiveForeground(LedState.kYellow.toString())
            .withText("MO");

    Command homeElevatorButtonCommand =
        elevator.manualHomeElevator().withName("homeElevatorButton");
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
            // .alongWith(Commands.runOnce(() -> Climber.setClimbState(ClimbState.DEPLOYING)))
            // .andThen(Commands.runOnce(() -> Climber.setClimbState(ClimbState.DEPLOYED)))
            .withName("climbDeployButton");
    Command climbClimbButtonCommand =
        climber
            .climbClimb()
            // .alongWith(Commands.runOnce(() -> Climber.setClimbState(ClimbState.CLIMBING)))
            // .andThen(Commands.runOnce(() -> Climber.setClimbState(ClimbState.CLIMBED)))
            .withName("climbClimbButton");
    Command manualClimbButtonCommand = climber.climbOut(12).withName("manualClimbButton");
    Command manualClimbOffButtonCommand = climber.climbSTOP().withName("manualClimbButtonOff");
    Command climbRollerStopButtonCommand =
        climbRoller.rollerSTOP().withName("climbRollerStopButton");
    Command driveToCoralButtonCommand = new DriveToCoralCommand(drive, vision);

    Map<StreamDeckButton, BooleanSupplier> customStreamDeckButtonMap = new HashMap<>();

    customStreamDeckButtonMap.put(
        coralL4Button,
        () -> robotState.getStoredScorePosition().getCoralScoreLevel() == ScoreLevel.L4);
    customStreamDeckButtonMap.put(
        coralL3Button,
        () -> robotState.getStoredScorePosition().getCoralScoreLevel() == ScoreLevel.L3);
    customStreamDeckButtonMap.put(
        coralL2Button,
        () -> robotState.getStoredScorePosition().getCoralScoreLevel() == ScoreLevel.L2);
    customStreamDeckButtonMap.put(
        coralL1Button,
        () -> robotState.getStoredScorePosition().getCoralScoreLevel() == ScoreLevel.L1);
    customStreamDeckButtonMap.put(
        AlgaeBargeButton,
        () -> robotState.getStoredScorePosition().getAlgaeScoreLevel() == ScoreLevel.BARGE);
    customStreamDeckButtonMap.put(
        AlgaeL2Button,
        () -> robotState.getStoredScorePosition().getAlgaeIntake() == AlgaeIntake.L2_ALGAE);
    customStreamDeckButtonMap.put(
        AlgaeL1Button,
        () -> robotState.getStoredScorePosition().getAlgaeIntake() == AlgaeIntake.L1_ALGAE);
    customStreamDeckButtonMap.put(
        AlgaeProcessorButton,
        () -> robotState.getStoredScorePosition().getAlgaeScoreLevel() == ScoreLevel.PROCESSOR);
    // customStreamDeckButtonMap.put(
    //     ReefASideButton, () -> robotState.getStoredScorePosition().getReefSide() == ReefSide.A);
    // customStreamDeckButtonMap.put(
    //     ReefBSideButton, () -> robotState.getStoredScorePosition().getReefSide() == ReefSide.B);
    // customStreamDeckButtonMap.put(
    //     ReefCSideButton, () -> robotState.getStoredScorePosition().getReefSide() == ReefSide.C);
    // customStreamDeckButtonMap.put(
    //     ReefDSideButton, () -> robotState.getStoredScorePosition().getReefSide() == ReefSide.D);
    // customStreamDeckButtonMap.put(
    //     ReefESideButton, () -> robotState.getStoredScorePosition().getReefSide() == ReefSide.E);
    // customStreamDeckButtonMap.put(
    //     ReefFSideButton, () -> robotState.getStoredScorePosition().getReefSide() == ReefSide.E);
    // customStreamDeckButtonMap.put(
    //     reefRightSideButton,
    //     () -> robotState.getStoredScorePosition().getCoralBranch() == CoralBranch.RIGHT);
    // customStreamDeckButtonMap.put(
    //     reefRightSideButton2,
    //     () -> robotState.getStoredScorePosition().getCoralBranch() == CoralBranch.RIGHT);
    // customStreamDeckButtonMap.put(
    //     reefLeftSideButton,
    //     () -> robotState.getStoredScorePosition().getCoralBranch() == CoralBranch.LEFT);
    // customStreamDeckButtonMap.put(
    //     reefLeftSideButton2,
    //     () -> robotState.getStoredScorePosition().getCoralBranch() == CoralBranch.LEFT);
    customStreamDeckButtonMap.put(homeElevatorButton, homeElevatorButtonCommand::isScheduled);
    customStreamDeckButtonMap.put(climbDeployButton, climbDeployButtonCommand::isScheduled);
    customStreamDeckButtonMap.put(climbDeployButton2, climbDeployButtonCommand::isScheduled);
    customStreamDeckButtonMap.put(climbRollerStopButton, climbRollerStopButtonCommand::isScheduled);
    customStreamDeckButtonMap.put(climbClimbButton, climbClimbButtonCommand::isScheduled);
    customStreamDeckButtonMap.put(climbClimbButton2, climbClimbButtonCommand::isScheduled);
    customStreamDeckButtonMap.put(manualClimbButton, manualClimbButtonCommand::isScheduled);
    customStreamDeckButtonMap.put(driveToCoralButton, driveToCoralButtonCommand::isScheduled);
    // customStreamDeckButtonMap.put(setManualScoringButton, () -> false);
    customStreamDeckButtonMap.put(
        manualOverrideButton, () -> RobotState.getSuperstructureManualOverrideMode());

    streamdeck.configureCustomButtons(customStreamDeckButtonMap);

    streamdeck.configureDefaultButtons(
        Set.of(
            zeroGyroButton,
            zeroGyroButton2,
            autoScoreLeftButton,
            autoScoreRightButton,
            autoScoreForwardButton,
            autoScoreBackButton,
            autoScoreZeroButton));

    streamdeck
        .button(coralL4Button)
        .onTrue(
            Commands.runOnce(
                    () -> robotState.getStoredScorePosition().setCoralScoreLevel(ScoreLevel.L4))
                .ignoringDisable(true));
    streamdeck
        .button(coralL3Button)
        .onTrue(
            Commands.runOnce(
                    () -> robotState.getStoredScorePosition().setCoralScoreLevel(ScoreLevel.L3))
                .ignoringDisable(true));
    streamdeck
        .button(coralL2Button)
        .onTrue(
            Commands.runOnce(
                    () -> robotState.getStoredScorePosition().setCoralScoreLevel(ScoreLevel.L2))
                .ignoringDisable(true));
    streamdeck
        .button(coralL1Button)
        .onTrue(
            Commands.runOnce(
                    () -> robotState.getStoredScorePosition().setCoralScoreLevel(ScoreLevel.L1))
                .ignoringDisable(true));
    streamdeck
        .button(AlgaeBargeButton)
        .onTrue(
            Commands.runOnce(
                    () -> robotState.getStoredScorePosition().setAlgaeScoreLevel(ScoreLevel.BARGE))
                .ignoringDisable(true));
    streamdeck
        .button(AlgaeL2Button)
        .onTrue(
            Commands.runOnce(
                    () -> {
                      ScorePosition stored = robotState.getStoredScorePosition();
                      if (stored.getAlgaeIntake() == AlgaeIntake.L2_ALGAE) {
                        stored.setAlgaeIntake(AlgaeIntake.NONE);
                      } else {
                        stored.setAlgaeIntake(AlgaeIntake.L2_ALGAE);
                      }
                    })
                .ignoringDisable(true));
    streamdeck
        .button(AlgaeL1Button)
        .onTrue(
            Commands.runOnce(
                    () -> {
                      ScorePosition stored = robotState.getStoredScorePosition();
                      if (stored.getAlgaeIntake() == AlgaeIntake.L1_ALGAE) {
                        stored.setAlgaeIntake(AlgaeIntake.NONE);
                      } else {
                        stored.setAlgaeIntake(AlgaeIntake.L1_ALGAE);
                      }
                    })
                .ignoringDisable(true));

    streamdeck
        .button(AlgaeProcessorButton)
        .onTrue(
            Commands.runOnce(
                    () ->
                        robotState
                            .getStoredScorePosition()
                            .setAlgaeScoreLevel(ScoreLevel.PROCESSOR))
                .ignoringDisable(true));
    // streamdeck
    //     .button(ReefASideButton)
    //     .onTrue(
    //         Commands.runOnce(() -> robotState.getStoredScorePosition().setReefSide(ReefSide.A))
    //             .ignoringDisable(true));
    // streamdeck
    //     .button(ReefBSideButton)
    //     .onTrue(
    //         Commands.runOnce(() -> robotState.getStoredScorePosition().setReefSide(ReefSide.B))
    //             .ignoringDisable(true));
    // streamdeck
    //     .button(ReefCSideButton)
    //     .onTrue(
    //         Commands.runOnce(() -> robotState.getStoredScorePosition().setReefSide(ReefSide.C))
    //             .ignoringDisable(true));
    // streamdeck
    //     .button(ReefDSideButton)
    //     .onTrue(
    //         Commands.runOnce(() -> robotState.getStoredScorePosition().setReefSide(ReefSide.D))
    //             .ignoringDisable(true));
    // streamdeck
    //     .button(ReefESideButton)
    //     .onTrue(
    //         Commands.runOnce(() -> robotState.getStoredScorePosition().setReefSide(ReefSide.E))
    //             .ignoringDisable(true));
    // streamdeck
    //     .button(ReefFSideButton)
    //     .onTrue(
    //         Commands.runOnce(() -> robotState.getStoredScorePosition().setReefSide(ReefSide.F))
    //             .ignoringDisable(true));
    // streamdeck
    //     .button(reefRightSideButton)
    //     .onTrue(
    //         Commands.runOnce(
    //                 () -> robotState.getStoredScorePosition().setBranchSide(CoralBranch.RIGHT))
    //             .ignoringDisable(true));
    // streamdeck
    //     .button(reefRightSideButton2)
    //     .onTrue(
    //         Commands.runOnce(
    //                 () -> robotState.getStoredScorePosition().setBranchSide(CoralBranch.RIGHT))
    //             .ignoringDisable(true));
    // streamdeck
    //     .button(reefLeftSideButton)
    //     .onTrue(
    //         Commands.runOnce(
    //                 () -> robotState.getStoredScorePosition().setBranchSide(CoralBranch.LEFT))
    //             .ignoringDisable(true));
    // streamdeck
    //     .button(reefLeftSideButton2)
    //     .onTrue(
    //         Commands.runOnce(
    //                 () -> robotState.getStoredScorePosition().setBranchSide(CoralBranch.LEFT))
    //             .ignoringDisable(true));
    streamdeck.button(homeElevatorButton).onTrue(homeElevatorButtonCommand);
    // streamdeck
    //     .button(setManualScoringButton)
    //     .onTrue(Commands.runOnce(() -> robotState.setScoringModeManual()));
    // streamdeck
    //     .button(setAutoScoringButton)
    //     .onTrue(Commands.runOnce(() -> robotState.setScoringModeAuto()));

    streamdeck
        .button(climbDeployButton)
        .and(streamdeck.button(climbDeployButton2))
        .onTrue(climbDeployButtonCommand);
    streamdeck
        .button(climbClimbButton)
        .and(streamdeck.button(climbClimbButton2))
        .and(streamdeck.button(manualClimbButton).negate())
        .onTrue(climbClimbButtonCommand);
    streamdeck
        .button(manualClimbButton)
        .and(streamdeck.button(climbClimbButton))
        .and(streamdeck.button(climbClimbButton2))
        .onTrue(manualClimbButtonCommand);
    streamdeck.button(climbDeployButton).onFalse(manualClimbOffButtonCommand);
    streamdeck.button(climbDeployButton2).onFalse(manualClimbOffButtonCommand);
    streamdeck.button(climbClimbButton).onFalse(manualClimbOffButtonCommand);
    streamdeck.button(climbClimbButton2).onFalse(manualClimbOffButtonCommand);
    streamdeck.button(manualClimbButton).onFalse(manualClimbOffButtonCommand);
    streamdeck.button(climbRollerStopButton).onTrue(climbRollerStopButtonCommand);
    streamdeck
        .button(zeroGyroButton)
        .and(streamdeck.button(zeroGyroButton2))
        .onTrue(
            Commands.runOnce(
                    () ->
                        drive.resetOdometry(
                            new Pose2d(
                                RobotState.getGlobalPose().getTranslation(), Rotation2d.kZero)),
                    drive)
                .ignoringDisable(true));
    streamdeck
        .button(manualOverrideButton)
        .onTrue(Commands.runOnce(() -> RobotState.toggleSuperstructureManualOverrideMode()));

    // Arbitrary triggers + streamdeck confirmation

    Trigger autoClimbTrigger = new Trigger(() -> climbRoller.hasCage()).debounce(.25);
    autoClimbTrigger.onTrue(climber.climbClimb().withName("AutoClimb"));
    streamdeck
        .button(climbDeployButton)
        .and(streamdeck.button(climbDeployButton2))
        .and(autoClimbTrigger)
        .onTrue(climber.climbClimb().withName("AutoClimb"));

    // left and right are swapped on purpose to match the operator's POV
    streamdeck
        .button(autoScoreLeftButton)
        .onTrue(Commands.runOnce(() -> robotState.offsetRight()).asProxy());
    streamdeck
        .button(autoScoreRightButton)
        .onTrue(Commands.runOnce(() -> robotState.offsetLeft()).asProxy());

    streamdeck
        .button(autoScoreForwardButton)
        .onTrue(Commands.runOnce(() -> robotState.offsetForward()).asProxy());
    streamdeck
        .button(autoScoreBackButton)
        .onTrue(Commands.runOnce(() -> robotState.offsetBackward()).asProxy());
    streamdeck
        .button(autoScoreZeroButton)
        .onTrue(Commands.runOnce(() -> robotState.offsetZero()).asProxy());
    streamdeck.button(driveToCoralButton).whileTrue(driveToCoralButtonCommand);
  }
}
