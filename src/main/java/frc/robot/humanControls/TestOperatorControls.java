package frc.robot.humanControls;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotContainer;
import frc.robot.RobotState;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.superstructure.Superstructure;
import frc.robot.subsystems.superstructure.SuperstructureState;
import frc.robot.util.Color;
import frc.robot.util.Controls.StreamDeck.StreamDeck;
import frc.robot.util.Controls.StreamDeck.StreamDeckButton;
import frc.robot.util.Controls.StreamDeck.StreamDeckButtonConfig;
import java.util.HashMap;
import java.util.Map;
import java.util.function.BooleanSupplier;

public class TestOperatorControls {
  private final RobotContainer container;
  private final Superstructure superstructure;
  private final Intake intake;
  private final StreamDeck streamdeck;
  private final RobotState robotState;

  public TestOperatorControls(
      RobotContainer container, StreamDeck streamdeck, RobotState robotState) {
    this.container = container;
    this.streamdeck = streamdeck;
    this.robotState = robotState;
    superstructure = container.getSuperStructure();
    intake = container.getIntake();
    configureTestingStreamDeckBindings();
  }

  private void configureTestingStreamDeckBindings() {
    StreamDeckButtonConfig orangeConfig =
        new StreamDeckButtonConfig(Color.kCOOrangePure.toString(), Color.kOff.toString(), "");
    StreamDeckButtonConfig tealConfig =
        new StreamDeckButtonConfig(Color.kCOTealPure.toString(), Color.kWhite.toString(), "");
    StreamDeckButtonConfig grayConfig =
        new StreamDeckButtonConfig(Color.kGray.toString(), Color.kWhite.toString(), "");
    StreamDeckButtonConfig redConfig =
        new StreamDeckButtonConfig(Color.kRed.toString(), Color.kOff.toString(), "");
    StreamDeckButtonConfig activeConfig =
        new StreamDeckButtonConfig(Color.kWhite.toString(), Color.kOff.toString(), "");

    // StreamDeckButton swerveXButton =
    //     new StreamDeckButton(3, 7, "Swerve X")
    //         .withInactiveConfig(inactiveConfig)
    //         .withActiveConfig(activeConfig)
    //         .withInactiveText("X")
    //         .withActiveText("Swerve X");
    StreamDeckButton intakeInButton =
        new StreamDeckButton(1, 0, "Intake In")
            .withInactiveConfig(orangeConfig)
            .withActiveConfig(activeConfig)
            .withText("INT In");
    StreamDeckButton intakeOutButton =
        new StreamDeckButton(1, 1, "Intake Out")
            .withInactiveConfig(orangeConfig)
            .withActiveConfig(activeConfig)
            .withText("INT Out");
    StreamDeckButton intakeUpButton =
        new StreamDeckButton(0, 0, "Intake Up")
            .withInactiveConfig(orangeConfig)
            .withActiveConfig(activeConfig)
            .withText("INT Up");
    StreamDeckButton intakeDownButton =
        new StreamDeckButton(0, 1, "Intake Down")
            .withInactiveConfig(orangeConfig)
            .withActiveConfig(activeConfig)
            .withText("INT Down");
    StreamDeckButton feederInButton =
        new StreamDeckButton(2, 0, "Feeder In ")
            .withInactiveConfig(orangeConfig)
            .withActiveConfig(activeConfig)
            .withText("FEED In");
    StreamDeckButton feederOutButton =
        new StreamDeckButton(2, 1, "Feeder Out")
            .withInactiveConfig(orangeConfig)
            .withActiveConfig(activeConfig)
            .withText("FEED Out ");
    StreamDeckButton intakePosScoreButton =
        new StreamDeckButton(1, 2, "Intake PosScore")
            .withInactiveConfig(orangeConfig)
            .withActiveConfig(activeConfig)
            .withText("INT PosScore");
    StreamDeckButton intakeDefaultButton =
        new StreamDeckButton(0, 3, "Intake Default")
            .withInactiveConfig(orangeConfig)
            .withActiveConfig(activeConfig)
            .withText("INT Default");
    StreamDeckButton intakeZeroButton =
        new StreamDeckButton(3, 2, "Intake Zero")
            .withInactiveConfig(redConfig)
            .withActiveConfig(activeConfig)
            .withText("INT Zero");
    StreamDeckButton noneButton =
        new StreamDeckButton(1, 3, "none")
            .withInactiveConfig(orangeConfig)
            .withActiveConfig(activeConfig)
            .withText("NONE");
    StreamDeckButton stowButton =
        new StreamDeckButton(3, 3, "stow")
            .withInactiveConfig(grayConfig)
            .withActiveConfig(activeConfig)
            .withText("STOW");
    StreamDeckButton stowCoralButton =
        new StreamDeckButton(2, 3, "stowCoral")
            .withInactiveConfig(grayConfig)
            .withActiveConfig(activeConfig)
            .withText("STOW C");
    StreamDeckButton stowAlgaeButton =
        new StreamDeckButton(1, 3, "stowAlgae")
            .withInactiveConfig(grayConfig)
            .withActiveConfig(activeConfig)
            .withText("STOW A");
    // StreamDeckButton intakeCoralButton =
    //     new StreamDeckButton(3,7, "intakeCoral")
    //         .withInactiveConfig(inactiveConfig)
    //         .withActiveConfig(activeConfig)
    //         .withText("intakeCoral");
    // StreamDeckButton intakeCoralL1Button =
    //     new StreamDeckButton(3,7, "intakeCoralL1")
    //         .withInactiveConfig(inactiveConfig)
    //         .withActiveConfig(activeConfig)
    //         .withText("intakeCoralL1");
    // StreamDeckButton feedButton =
    //     new StreamDeckButton(3,7, "feed")
    //         .withInactiveConfig(inactiveConfig)
    //         .withActiveConfig(activeConfig)
    //         .withText("feed");
    StreamDeckButton l1FadeawayButton =
        new StreamDeckButton(3, 5, "l1Fadeaway")
            .withInactiveConfig(orangeConfig)
            .withActiveConfig(activeConfig)
            .withText("L1 F");
    StreamDeckButton l2FadeawayButton =
        new StreamDeckButton(2, 5, "l2Fadeaway")
            .withInactiveConfig(orangeConfig)
            .withActiveConfig(activeConfig)
            .withText("L2 F");
    StreamDeckButton l3FadeawayButton =
        new StreamDeckButton(1, 5, "l3Fadeaway")
            .withInactiveConfig(orangeConfig)
            .withActiveConfig(activeConfig)
            .withText("L3 F");
    StreamDeckButton l4FadeawayButton =
        new StreamDeckButton(0, 5, "l4Fadeaway")
            .withInactiveConfig(orangeConfig)
            .withActiveConfig(activeConfig)
            .withText("L4 F");
    StreamDeckButton l1ScoreButton =
        new StreamDeckButton(3, 4, "l1Score")
            .withInactiveConfig(orangeConfig)
            .withActiveConfig(activeConfig)
            .withText("L1 S");
    StreamDeckButton l2ScoreButton =
        new StreamDeckButton(2, 4, "l2Score")
            .withInactiveConfig(orangeConfig)
            .withActiveConfig(activeConfig)
            .withText("L2 S");
    StreamDeckButton l3ScoreButton =
        new StreamDeckButton(1, 4, "l3Score")
            .withInactiveConfig(orangeConfig)
            .withActiveConfig(activeConfig)
            .withText("L3 S");
    StreamDeckButton l4ScoreButton =
        new StreamDeckButton(0, 4, "l4Score")
            .withInactiveConfig(orangeConfig)
            .withActiveConfig(activeConfig)
            .withText("L4 S");
    StreamDeckButton algaeHighIntakeButton =
        new StreamDeckButton(1, 6, "algaeHighIntake")
            .withInactiveConfig(tealConfig)
            .withActiveConfig(activeConfig)
            .withText("ALG H");
    StreamDeckButton algaeLowIntakeButton =
        new StreamDeckButton(2, 6, "algaeLowIntake")
            .withInactiveConfig(tealConfig)
            .withActiveConfig(activeConfig)
            .withText("ALG L");
    StreamDeckButton processorAimButton =
        new StreamDeckButton(3, 7, "processorAim")
            .withInactiveConfig(tealConfig)
            .withActiveConfig(activeConfig)
            .withText("P");
    StreamDeckButton bargeAimCenterButton =
        new StreamDeckButton(1, 7, "bargeAimCenter")
            .withInactiveConfig(tealConfig)
            .withActiveConfig(activeConfig)
            .withText("B-C");
    StreamDeckButton bargeAimForwardButton =
        new StreamDeckButton(2, 7, "bargeAimForward")
            .withInactiveConfig(tealConfig)
            .withActiveConfig(activeConfig)
            .withText("B-F");
    StreamDeckButton bargeAimBackwardButton =
        new StreamDeckButton(0, 7, "bargeAimBackward")
            .withInactiveConfig(tealConfig)
            .withActiveConfig(activeConfig)
            .withText("B-B");

    // Command swerveXButtonCommand =
    // Commands.runOnce(drive::stopWithX, drive).withName("swerveXButton");
    Command intakeInButtonCommand = intake.intakeFWD().withName("intakeInButton");
    Command intakeInButtonOffCommand = intake.intakeSTOP().withName("intakeInButtonOff");
    Command intakeOutButtonCommand = intake.intakeRVS().withName("intakeOutButton");
    Command intakeOutButtonOffCommand = intake.intakeSTOP().withName("intakeOutButtonOff");
    Command intakeUpButtonCommand = intake.pivotManualTestForward().withName("intakeUpButton");
    Command intakeUpButtonOffCommand = intake.pivotStop().withName("intakeUpButtonOff");
    Command intakeDownButtonCommand = intake.pivotManualTestReverse().withName("intakeDownButton");
    Command intakeDownButtonOffCommand = intake.pivotStop().withName("intakeDownButtonOff");
    // Command intakeL1UpButtonCommand =
    //     intake.().withName("intakeL1UpButton");
    Command feederInButtonCommand = intake.feederFWD().withName("feederInButton");
    Command feederInButtonOffCommand = intake.feederSTOP().withName("feederInButtonOff");
    Command feederOutButtonCommand = intake.feederRVS().withName("feederOutButton");
    Command feederOutButtonOffCommand = intake.feederSTOP().withName("feederOutButtonOff");
    // Command intakePosUpButtonCommand = intake.setPivotUp().withName("intakePosUpButton");
    // Command intakePosDownButtonCommand = intake.setPivotDown().withName("intakePosDownButton");
    Command intakePosScoreButtonCommand = intake.setPivotScoring().withName("intakePosScoreButton");
    Command intakeDefaultButtonCommand =
        Commands.runOnce(() -> CommandScheduler.getInstance().cancel(intake.getCurrentCommand()))
            .withName("intakeDefaultButton");
    Command intakeZeroButtonCommand = intake.zeroPivotAtPivotUp().withName("intakeZeroButton");

    // streamdeck.configureDefaultButtons(
    //     Set.of(
    //         swerveXButton,
    //         intakeInButton,
    //         intakeOutButton,
    //         intakeUpButton,
    //         intakeDownButton,
    //         intakeL1UpButton,
    //         intakeL1DownButton,
    //         feederInButton,
    //         feederOutButton,
    //         intakeZeroButton));

    // streamdeck.configureToggleButtons(
    //     Set.of(intakePosUpButton, intakePosDownButton, intakePosScoreButton));

    Map<StreamDeckButton, BooleanSupplier> customStreamDeckButtonMap = new HashMap<>();

    // customStreamDeckButtonMap.put(swerveXButton, swerveXButtonCommand::isScheduled);
    customStreamDeckButtonMap.put(intakeInButton, intakeInButtonCommand::isScheduled);
    customStreamDeckButtonMap.put(intakeOutButton, intakeOutButtonCommand::isScheduled);
    customStreamDeckButtonMap.put(intakeUpButton, intakeUpButtonCommand::isScheduled);
    customStreamDeckButtonMap.put(intakeDownButton, intakeDownButtonCommand::isScheduled);
    // customStreamDeckButtonMap.put(intakeL1UpButton, intakeL1UpButtonCommand::isScheduled);
    customStreamDeckButtonMap.put(feederInButton, feederInButtonCommand::isScheduled);
    customStreamDeckButtonMap.put(feederOutButton, feederOutButtonCommand::isScheduled);
    // customStreamDeckButtonMap.put(intakePosUpButton, intakePosUpButtonCommand::isScheduled);
    // customStreamDeckButtonMap.put(intakePosDownButton, intakePosDownButtonCommand::isScheduled);
    customStreamDeckButtonMap.put(intakePosScoreButton, intakePosScoreButtonCommand::isScheduled);
    customStreamDeckButtonMap.put(intakeDefaultButton, intake.getDefaultCommand()::isScheduled);
    customStreamDeckButtonMap.put(intakeZeroButton, intakeZeroButtonCommand::isScheduled);

    customStreamDeckButtonMap.put(
        noneButton, () -> superstructure.getCurrentState() == SuperstructureState.NONE);
    customStreamDeckButtonMap.put(
        stowButton, () -> superstructure.getCurrentState() == SuperstructureState.STOW);
    customStreamDeckButtonMap.put(
        stowCoralButton, () -> superstructure.getCurrentState() == SuperstructureState.STOW_CORAL);
    customStreamDeckButtonMap.put(
        stowAlgaeButton, () -> superstructure.getCurrentState() == SuperstructureState.STOW_ALGAE);
    // customStreamDeckButtonMap.put(intakeCoralButton, () -> superstructure.getCurrentState() ==
    // SuperstructureState.INTAKE_CORAL);
    // customStreamDeckButtonMap.put(intakeCoralL1Button, () -> superstructure.getCurrentState() ==
    // SuperstructureState.INTAKE_CORAL_L1);
    // customStreamDeckButtonMap.put(feedButton, () -> superstructure.getCurrentState() ==
    // SuperstructureState.FEED);
    customStreamDeckButtonMap.put(
        l1FadeawayButton,
        () -> superstructure.getCurrentState() == SuperstructureState.L1_FADEAWAY);
    customStreamDeckButtonMap.put(
        l2FadeawayButton,
        () -> superstructure.getCurrentState() == SuperstructureState.L2_FADEAWAY);
    customStreamDeckButtonMap.put(
        l3FadeawayButton,
        () -> superstructure.getCurrentState() == SuperstructureState.L3_FADEAWAY);
    customStreamDeckButtonMap.put(
        l4FadeawayButton,
        () -> superstructure.getCurrentState() == SuperstructureState.L4_FADEAWAY);
    customStreamDeckButtonMap.put(
        l1ScoreButton, () -> superstructure.getCurrentState() == SuperstructureState.L1_AIM);
    customStreamDeckButtonMap.put(
        l2ScoreButton, () -> superstructure.getCurrentState() == SuperstructureState.L2_AIM);
    customStreamDeckButtonMap.put(
        l3ScoreButton, () -> superstructure.getCurrentState() == SuperstructureState.L3_AIM);
    customStreamDeckButtonMap.put(
        l4ScoreButton, () -> superstructure.getCurrentState() == SuperstructureState.L4_AIM);
    customStreamDeckButtonMap.put(
        algaeHighIntakeButton,
        () -> superstructure.getCurrentState() == SuperstructureState.ALGAE_HIGH_INTAKE);
    customStreamDeckButtonMap.put(
        algaeLowIntakeButton,
        () -> superstructure.getCurrentState() == SuperstructureState.ALGAE_LOW_INTAKE);
    customStreamDeckButtonMap.put(
        processorAimButton,
        () -> superstructure.getCurrentState() == SuperstructureState.PROCESSOR_AIM);
    customStreamDeckButtonMap.put(
        bargeAimCenterButton,
        () -> superstructure.getCurrentState() == SuperstructureState.BARGE_AIM_CENTER);
    customStreamDeckButtonMap.put(
        bargeAimForwardButton,
        () -> superstructure.getCurrentState() == SuperstructureState.BARGE_AIM_FORWARD);
    customStreamDeckButtonMap.put(
        bargeAimBackwardButton,
        () -> superstructure.getCurrentState() == SuperstructureState.BARGE_AIM_BACKWARD);

    streamdeck.configureCustomButtons(customStreamDeckButtonMap);

    // streamdeck.button(swerveXButton).onTrue(swerveXButtonCommand);
    streamdeck.button(intakeInButton).whileTrue(intakeInButtonCommand);
    streamdeck.button(intakeInButton).onFalse(intakeInButtonOffCommand);
    streamdeck.button(intakeOutButton).whileTrue(intakeOutButtonCommand);
    streamdeck.button(intakeOutButton).onFalse(intakeOutButtonOffCommand);
    streamdeck.button(intakeUpButton).whileTrue(intakeUpButtonCommand);
    streamdeck.button(intakeUpButton).onFalse(intakeUpButtonOffCommand);
    streamdeck.button(intakeDownButton).whileTrue(intakeDownButtonCommand);
    streamdeck.button(intakeDownButton).onFalse(intakeDownButtonOffCommand);
    // streamdeck.button(intakeL1UpButton).whileTrue(intakeL1UpButtonCommand);
    streamdeck.button(feederInButton).whileTrue(feederInButtonCommand);
    streamdeck.button(feederInButton).onFalse(feederInButtonOffCommand);
    streamdeck.button(feederOutButton).whileTrue(feederOutButtonCommand);
    streamdeck.button(feederOutButton).onFalse(feederOutButtonOffCommand);
    // streamdeck.button(intakePosUpButton).onTrue(intakePosUpButtonCommand);
    // streamdeck.button(intakePosDownButton).onTrue(intakePosDownButtonCommand);
    streamdeck.button(intakePosScoreButton).onTrue(intakePosScoreButtonCommand);
    streamdeck.button(intakeDefaultButton).onTrue(intakeDefaultButtonCommand);
    streamdeck.button(intakeZeroButton).onTrue(intakeZeroButtonCommand);

    streamdeck
        .button(noneButton)
        .onTrue(superstructure.setStateCommand(SuperstructureState.NONE, "Set NONE"));
    streamdeck
        .button(stowButton)
        .onTrue(superstructure.setStateCommand(SuperstructureState.STOW, "Set STOW"));
    streamdeck
        .button(stowCoralButton)
        .onTrue(superstructure.setStateCommand(SuperstructureState.STOW_CORAL, "Set STOW_CORAL"));
    streamdeck
        .button(stowAlgaeButton)
        .onTrue(superstructure.setStateCommand(SuperstructureState.STOW_ALGAE, "Set STOW_ALGAE"));
    // streamdeck.button(intakeCoralButton).onTrue(superstructure.setStateCommand(SuperstructureState.INTAKE_CORAL, "Set INTAKE_CORAL"));
    // streamdeck.button(intakeCoralL1Button).onTrue(superstructure.setStateCommand(SuperstructureState.INTAKE_CORAL_L1, "Set INTAKE_CORAL_L1"));
    // streamdeck.button(feedButton).onTrue(superstructure.setStateCommand(SuperstructureState.FEED,
    // "Set FEED"));
    streamdeck
        .button(l1FadeawayButton)
        .onTrue(superstructure.setStateCommand(SuperstructureState.L1_FADEAWAY, "Set L1_FADEAWAY"));
    streamdeck
        .button(l2FadeawayButton)
        .onTrue(superstructure.setStateCommand(SuperstructureState.L2_FADEAWAY, "Set L2_FADEAWAY"));
    streamdeck
        .button(l3FadeawayButton)
        .onTrue(superstructure.setStateCommand(SuperstructureState.L3_FADEAWAY, "Set L3_FADEAWAY"));
    streamdeck
        .button(l4FadeawayButton)
        .onTrue(superstructure.setStateCommand(SuperstructureState.L4_FADEAWAY, "Set L4_FADEAWAY"));
    streamdeck
        .button(l1ScoreButton)
        .onTrue(superstructure.setStateCommand(SuperstructureState.L1_AIM, "Set L1_AIM"));
    streamdeck
        .button(l2ScoreButton)
        .onTrue(superstructure.setStateCommand(SuperstructureState.L2_AIM, "Set L2_AIM"));
    streamdeck
        .button(l3ScoreButton)
        .onTrue(superstructure.setStateCommand(SuperstructureState.L3_AIM, "Set L3_AIM"));
    streamdeck
        .button(l4ScoreButton)
        .onTrue(superstructure.setStateCommand(SuperstructureState.L4_AIM, "Set L4_AIM"));
    streamdeck
        .button(algaeHighIntakeButton)
        .onTrue(
            superstructure.setStateCommand(
                SuperstructureState.ALGAE_HIGH_INTAKE, "Set ALGAE_HIGH_INTAKE"));
    streamdeck
        .button(algaeLowIntakeButton)
        .onTrue(
            superstructure.setStateCommand(
                SuperstructureState.ALGAE_LOW_INTAKE, "Set ALGAE_LOW_INTAKE"));
    streamdeck
        .button(processorAimButton)
        .onTrue(
            superstructure.setStateCommand(SuperstructureState.PROCESSOR_AIM, "Set PROCESSOR_AIM"));
    streamdeck
        .button(bargeAimCenterButton)
        .onTrue(
            superstructure.setStateCommand(
                SuperstructureState.BARGE_AIM_CENTER, "Set BARGE_AIM_CENTER"));
    streamdeck
        .button(bargeAimForwardButton)
        .onTrue(
            superstructure.setStateCommand(
                SuperstructureState.BARGE_AIM_FORWARD, "Set BARGE_AIM_FORWARD"));
    streamdeck
        .button(bargeAimBackwardButton)
        .onTrue(
            superstructure.setStateCommand(
                SuperstructureState.BARGE_AIM_BACKWARD, "Set BARGE_AIM_BACKWARD"));
  }
}
