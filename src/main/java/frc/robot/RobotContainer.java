// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.test.DrivetrainTest;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.climb.Climber;
import frc.robot.subsystems.climb.ClimberIO;
import frc.robot.subsystems.climb.ClimberIOReal;
import frc.robot.subsystems.climb.ClimberIOSim;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorIO;
import frc.robot.subsystems.elevator.ElevatorIOReal;
import frc.robot.subsystems.elevator.ElevatorIOSim;
import frc.robot.subsystems.end_effector.Claw;
import frc.robot.subsystems.end_effector.ClawIO;
import frc.robot.subsystems.end_effector.ClawIOReal;
import frc.robot.subsystems.end_effector.ClawIOSim;
import frc.robot.subsystems.end_effector.EndEffector;
import frc.robot.subsystems.end_effector.EndEffectorIO;
import frc.robot.subsystems.end_effector.EndEffectorIOReal;
import frc.robot.subsystems.end_effector.EndEffectorIOSim;
import frc.robot.subsystems.feeder.Feeder;
import frc.robot.subsystems.feeder.FeederIO;
import frc.robot.subsystems.feeder.FeederIOReal;
import frc.robot.subsystems.feeder.FeederIOSim;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeIO;
import frc.robot.subsystems.intake.IntakeIOReal;
import frc.robot.subsystems.intake.IntakeIOSim;
import frc.robot.subsystems.led.LedState;
import frc.robot.subsystems.superstructure.Superstructure;
import frc.robot.subsystems.superstructure.SuperstructureState;
import frc.robot.util.Controls.StreamDeck;
import frc.robot.util.Controls.StreamDeckButton;
import frc.robot.util.Controls.StreamDeckButtonConfig;
import frc.robot.Constants.IntakeConstants.IntakeState;
import java.util.Set;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  private final Drive drive;
  private final Intake intake;
  private final EndEffector endEffector;
  private final Claw claw;
  private final Elevator elevator;
  private final Superstructure superstructure;
  private final Climber climber;
  private final Feeder feeder;

  // Controller
  private final CommandXboxController controller = new CommandXboxController(0);
  private final StreamDeck streamdeck = new StreamDeck();

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        feeder = new Feeder(new FeederIOReal());
        intake = new Intake(new IntakeIOReal(), feeder);
        endEffector = new EndEffector(new EndEffectorIOReal());
        claw = new Claw(new ClawIOReal() {});
        elevator = new Elevator(new ElevatorIOReal());
        superstructure = new Superstructure(elevator, endEffector);
        climber = new Climber(new ClimberIOReal());
        drive =
            new Drive(
                new GyroIOPigeon2(),
                new ModuleIOTalonFX(TunerConstants.FrontLeft),
                new ModuleIOTalonFX(TunerConstants.FrontRight),
                new ModuleIOTalonFX(TunerConstants.BackLeft),
                new ModuleIOTalonFX(TunerConstants.BackRight)
                // ,superstructure
                );
        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        feeder = new Feeder(new FeederIOSim());
        intake = new Intake(new IntakeIOSim(), feeder);
        endEffector = new EndEffector(new EndEffectorIOSim());
        elevator = new Elevator(new ElevatorIOSim());
        claw = new Claw(new ClawIOSim() {});
        superstructure = new Superstructure(elevator, endEffector);
        climber = new Climber(new ClimberIOSim());
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIOSim(TunerConstants.FrontLeft),
                new ModuleIOSim(TunerConstants.FrontRight),
                new ModuleIOSim(TunerConstants.BackLeft),
                new ModuleIOSim(TunerConstants.BackRight)
                // ,superstructure
                );
        break;

      default:
        // Replayed robot, disable IO implementations
        feeder = new Feeder(new FeederIO() {});
        intake = new Intake(new IntakeIO() {}, feeder);
        endEffector = new EndEffector(new EndEffectorIO() {});
        claw = new Claw(new ClawIO() {});
        elevator = new Elevator(new ElevatorIO() {});
        superstructure = new Superstructure(elevator, endEffector);
        climber = new Climber(new ClimberIO() {});
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {}
                // ,superstructure
                );
        break;
    }

    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    // Set up SysId routines
    autoChooser.addOption(
        "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
    autoChooser.addOption(
        "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
    autoChooser.addOption(
        "Drive Slip Current Characterization (Wall Test)",
        DriveCommands.slipCurrentCharacterization(drive));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Forward)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Reverse)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));

    autoChooser.addOption("Drivetrain Test", new DrivetrainTest(drive));

    // Configure default commands for subsystems
    RegisterDefaultCommands();

    // Build elastic tabs for testing
    buildElasticTabs();

    // Configure the button bindings
    configureButtonBindings();

    // Configure arbitrary triggers
    configureArbitraryTriggers();
  }

  private void configureButtonBindings() {
    configureXboxBindings();
    configureStreamDeckBindings();
  }

  private void RegisterDefaultCommands() {
    // Default command, normal field-relative drive
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -controller.getLeftY(),
            () -> -controller.getLeftX(),
            () -> -controller.getRightX()));
    elevator.setDefaultCommand(defaultElevatorCommand());
    endEffector.setDefaultCommand(defaultEndEffectorCommand());
    intake.setDefaultCommand(intake.intakeDefault());
  }

  private void buildElasticTabs() {
    buildIntakeTab();
    buildEndEffectorTab();
    buildElevatorTab();
    buildSuperstructureTab();
    buildClimberTab();
    buildDriveTab();
  }

  private void buildIntakeTab() {
    // Get the NetworkTable for the Intake tab
    NetworkTable intakeTable = NetworkTableInstance.getDefault().getTable("Intake");

    // Create NetworkTableEntry instances for while-held functionality
    NetworkTableEntry intakeForwardEntry = intakeTable.getEntry("Roller Forward (While Held)");
    NetworkTableEntry intakeReverseEntry = intakeTable.getEntry("Roller Reverse (While Held)");

    NetworkTableEntry intakeUpEntry = intakeTable.getEntry("Pivot Up (While Held)");
    NetworkTableEntry intakeDownEntry = intakeTable.getEntry("Pivot Down (While Held)");

    NetworkTableEntry intakeUpPosEntry = intakeTable.getEntry("Pivot Up (When Pressed)");
    NetworkTableEntry intakeDownPosEntry =
        intakeTable.getEntry("Pivot Intake Position (When Pressed)");
    NetworkTableEntry intakeScoringPosEntry =
        intakeTable.getEntry("Pivot Scoring Position (When Pressed)");
    NetworkTableEntry intakeZeroPosEntry =
        intakeTable.getEntry("Pivot Zero Position (When Pressed)");

    NetworkTableEntry l1BarUpEntry = intakeTable.getEntry("L1 Bar Up (While Held)");
    NetworkTableEntry l1BarDownEntry = intakeTable.getEntry("L1 Bar Down (While Held)");

    NetworkTableEntry feederForwardEntry = intakeTable.getEntry("Feeder In (While Held)");
    NetworkTableEntry feederReverseEntry = intakeTable.getEntry("Feeder Out (While Held)");

    // Intake State buttons
    NetworkTableEntry intakeStateStowEntry = intakeTable.getEntry("STOW (When Pressed)");
    NetworkTableEntry intakeStateIntakeL1Entry = intakeTable.getEntry("L1 (When Pressed)");
    NetworkTableEntry intakeStateIntakeEntry = intakeTable.getEntry("Intake (When Pressed)");
    NetworkTableEntry intakeStateRejectCoralEntry = intakeTable.getEntry("Reject Coral (When Pressed)");
    NetworkTableEntry intakeStateIdleEntry = intakeTable.getEntry("Idle (When Pressed)");
    NetworkTableEntry intakeStateHandOffEntry = intakeTable.getEntry("Hand Off (When Pressed)");
    NetworkTableEntry intakeStateScoringEntry = intakeTable.getEntry("Scoring (When Pressed)");
    NetworkTableEntry intakeStateScoringPrepEntry = intakeTable.getEntry("Scoring Prep (When Pressed)");

    // Initialize entries with default values
    intakeForwardEntry.setBoolean(false);
    intakeReverseEntry.setBoolean(false);

    intakeUpEntry.setBoolean(false);
    intakeDownEntry.setBoolean(false);

    intakeUpPosEntry.setBoolean(false);
    intakeDownPosEntry.setBoolean(false);
    intakeScoringPosEntry.setBoolean(false);
    intakeZeroPosEntry.setBoolean(false);

    l1BarUpEntry.setBoolean(false);
    l1BarDownEntry.setBoolean(false);

    feederForwardEntry.setBoolean(false);
    feederReverseEntry.setBoolean(false);

    // Initialize intake state entries
    intakeStateStowEntry.setBoolean(false);
    intakeStateIntakeL1Entry.setBoolean(false);
    intakeStateIntakeEntry.setBoolean(false);
    intakeStateRejectCoralEntry.setBoolean(false);
    intakeStateIdleEntry.setBoolean(false);
    intakeStateHandOffEntry.setBoolean(false);
    intakeStateScoringEntry.setBoolean(false);
    intakeStateScoringPrepEntry.setBoolean(false);

    // Create triggers based on the NetworkTableEntry values
    Trigger intakeForwardTrigger = new Trigger(() -> intakeForwardEntry.getBoolean(false));
    Trigger intakeReverseTrigger = new Trigger(() -> intakeReverseEntry.getBoolean(false));

    Trigger intakeUpTrigger = new Trigger(() -> intakeUpEntry.getBoolean(false));
    Trigger intakeDownTrigger = new Trigger(() -> intakeDownEntry.getBoolean(false));

    Trigger intakeUpPosTrigger = new Trigger(() -> intakeUpPosEntry.getBoolean(false));
    Trigger intakeDownPosTrigger = new Trigger(() -> intakeDownPosEntry.getBoolean(false));
    Trigger intakeScoringPosTrigger = new Trigger(() -> intakeScoringPosEntry.getBoolean(false));
    Trigger intakeZeroPosTrigger = new Trigger(() -> intakeZeroPosEntry.getBoolean(false));

    Trigger l1BarUpTrigger = new Trigger(() -> l1BarUpEntry.getBoolean(false));
    Trigger l1BarDownTrigger = new Trigger(() -> l1BarDownEntry.getBoolean(false));

    Trigger feederInTrigger = new Trigger(() -> feederForwardEntry.getBoolean(false));
    Trigger feederOutTrigger = new Trigger(() -> feederReverseEntry.getBoolean(false));

    // Create triggers for intake state buttons
    Trigger intakeStateStowTrigger = new Trigger(() -> intakeStateStowEntry.getBoolean(false));
    Trigger intakeStateIntakeL1Trigger = new Trigger(() -> intakeStateIntakeL1Entry.getBoolean(false));
    Trigger intakeStateIntakeTrigger = new Trigger(() -> intakeStateIntakeEntry.getBoolean(false));
    Trigger intakeStateRejectCoralTrigger = new Trigger(() -> intakeStateRejectCoralEntry.getBoolean(false));
    Trigger intakeStateIdleTrigger = new Trigger(() -> intakeStateIdleEntry.getBoolean(false));
    Trigger intakeStateHandOffTrigger = new Trigger(() -> intakeStateHandOffEntry.getBoolean(false));
    Trigger intakeStateScoringTrigger = new Trigger(() -> intakeStateScoringEntry.getBoolean(false));
    Trigger intakeStateScoringPrepTrigger = new Trigger(() -> intakeStateScoringPrepEntry.getBoolean(false));

    // Configure the while-held behavior
    intakeForwardTrigger.whileTrue(intake.intakeFWD());
    intakeForwardTrigger.onFalse(intake.intakeSTOP());

    intakeReverseTrigger.whileTrue(intake.intakeRVS());
    intakeReverseTrigger.onFalse(intake.intakeSTOP());

    intakeUpTrigger.whileTrue(intake.pivotManualTestForward());
    intakeUpTrigger.onFalse(intake.pivotStop());

    intakeDownTrigger.whileTrue(intake.pivotManualTestReverse());
    intakeDownTrigger.onFalse(intake.pivotStop());

    l1BarUpTrigger.whileTrue(intake.l1BarFWD());
    l1BarUpTrigger.onFalse(intake.l1BarSTOP());

    l1BarDownTrigger.whileTrue(intake.l1BarRVS());
    l1BarDownTrigger.onFalse(intake.l1BarSTOP());

    feederInTrigger.whileTrue(intake.feederFWD());
    feederInTrigger.onFalse(intake.feederSTOP());

    feederOutTrigger.whileTrue(intake.feederRVS());
    feederOutTrigger.onFalse(intake.feederSTOP());

    intakeUpPosTrigger.onTrue(intake.setPivotUp().andThen(() -> intakeUpPosEntry.setBoolean(false)));
    intakeDownPosTrigger.onTrue(intake.movePivotDown().andThen(() -> intakeDownPosEntry.setBoolean(false)));
    intakeScoringPosTrigger.onTrue(intake.setPivotScoring().andThen(() -> intakeScoringPosEntry.setBoolean(false)));
    intakeZeroPosTrigger.onTrue(intake.zeroPivotAtPivotUp().andThen(() -> intakeZeroPosEntry.setBoolean(false)));

    // Configure intake state button triggers
    intakeStateStowTrigger.onTrue(
        intake.setIntakeState(IntakeState.STOW)
            .andThen(() -> intakeStateStowEntry.setBoolean(false)));
    intakeStateIntakeL1Trigger.onTrue(
        intake.setIntakeState(IntakeState.INTAKE_L1)
            .andThen(() -> intakeStateIntakeL1Entry.setBoolean(false)));
    intakeStateIntakeTrigger.onTrue(
        intake.setIntakeState(IntakeState.INTAKE)
            .andThen(() -> intakeStateIntakeEntry.setBoolean(false)));
    intakeStateRejectCoralTrigger.onTrue(
        intake.setIntakeState(IntakeState.REJECT_CORAL)
            .andThen(() -> intakeStateRejectCoralEntry.setBoolean(false)));
    intakeStateIdleTrigger.onTrue(
        intake.setIntakeState(IntakeState.IDLE)
            .andThen(() -> intakeStateIdleEntry.setBoolean(false)));
    intakeStateHandOffTrigger.onTrue(
        intake.setIntakeState(IntakeState.HAND_OFF)
            .andThen(() -> intakeStateHandOffEntry.setBoolean(false)));
    intakeStateScoringTrigger.onTrue(
        intake.setIntakeState(IntakeState.SCORING)
            .andThen(() -> intakeStateScoringEntry.setBoolean(false)));
    intakeStateScoringPrepTrigger.onTrue(
        intake.setIntakeState(IntakeState.SCORING_PREP)
            .andThen(() -> intakeStateScoringPrepEntry.setBoolean(false)));
  }

  private void buildEndEffectorTab() {
    // Get the NetworkTable for the EndEffector tab
    NetworkTable endEffectorTable = NetworkTableInstance.getDefault().getTable("EndEffector");

    // Create NetworkTableEntry instances for while-held functionality
    NetworkTableEntry clawForwardEntry = endEffectorTable.getEntry("Roller Forward (While Held)");
    NetworkTableEntry clawReverseEntry = endEffectorTable.getEntry("Roller Reverse (While Held)");
    NetworkTableEntry pivotUpEntry = endEffectorTable.getEntry("Pivot Up (While Held)");
    NetworkTableEntry pivotDownEntry = endEffectorTable.getEntry("Pivot Down (While Held)");

    NetworkTableEntry pivotSafeUpEntry = endEffectorTable.getEntry("Pivot Safe Up (When Pressed)");
    NetworkTableEntry pivotUpPosEntry = endEffectorTable.getEntry("Pivot Fully Up (When Pressed)");
    NetworkTableEntry pivotSafeDownPosEntry =
        endEffectorTable.getEntry("Pivot Safe Down (When Pressed)");
    NetworkTableEntry pivotDownPosEntry =
        endEffectorTable.getEntry("Pivot Fully Down (When Pressed)");
    NetworkTableEntry pivotMiddlePosEntry =
        endEffectorTable.getEntry("Pivot Middle (When Pressed)");
    NetworkTableEntry pivotManualZeroEntry = endEffectorTable.getEntry("Pivot Zero (When Pressed)");

    // Initialize entries with default values
    clawForwardEntry.setBoolean(false);
    clawReverseEntry.setBoolean(false);
    pivotUpEntry.setBoolean(false);
    pivotDownEntry.setBoolean(false);

    pivotSafeUpEntry.setBoolean(false);
    pivotUpPosEntry.setBoolean(false);
    pivotSafeDownPosEntry.setBoolean(false);
    pivotDownPosEntry.setBoolean(false);
    pivotMiddlePosEntry.setBoolean(false);

    pivotManualZeroEntry.setBoolean(false);

    // Create triggers based on the NetworkTableEntry values
    Trigger clawForwardTrigger = new Trigger(() -> clawForwardEntry.getBoolean(false));
    Trigger clawReverseTrigger = new Trigger(() -> clawReverseEntry.getBoolean(false));
    Trigger pivotUpTrigger = new Trigger(() -> pivotUpEntry.getBoolean(false));
    Trigger pivotDownTrigger = new Trigger(() -> pivotDownEntry.getBoolean(false));

    Trigger pivotUpPosTrigger = new Trigger(() -> pivotUpPosEntry.getBoolean(false));
    Trigger pivotSafeUpPosTrigger = new Trigger(() -> pivotSafeUpEntry.getBoolean(false));
    Trigger pivotDownPosTrigger = new Trigger(() -> pivotDownPosEntry.getBoolean(false));
    Trigger pivotSafeDownPosTrigger = new Trigger(() -> pivotSafeDownPosEntry.getBoolean(false));
    Trigger pivotMiddlePosTrigger = new Trigger(() -> pivotMiddlePosEntry.getBoolean(false));

    Trigger pivotManualZeroTrigger = new Trigger(() -> pivotManualZeroEntry.getBoolean(false));

    // Configure the while-held behavior
    clawForwardTrigger.whileTrue(claw.rollerFWD());
    clawForwardTrigger.onFalse(claw.rollerSTOP());

    clawReverseTrigger.whileTrue(claw.rollerRVS());
    clawReverseTrigger.onFalse(claw.rollerSTOP());

    pivotUpTrigger.whileTrue(endEffector.pivotUP());
    pivotUpTrigger.onFalse(endEffector.pivotSTOP());

    pivotDownTrigger.whileTrue(endEffector.pivotDOWN());
    pivotDownTrigger.onFalse(endEffector.pivotSTOP());

    pivotUpPosTrigger.onTrue(
        endEffector.rotatePivot(() -> Constants.EndEffectorConstants.MAX_ANGLE_RADIAN).andThen(() -> pivotUpPosEntry.setBoolean(false)));
    pivotSafeUpPosTrigger.onTrue(
        endEffector.rotatePivot(() -> Constants.EndEffectorConstants.MAX_SAFE_ANGLE_RADIAN).andThen(() -> pivotSafeUpEntry.setBoolean(false)));
    pivotSafeDownPosTrigger.onTrue(
        endEffector.rotatePivot(() -> Constants.EndEffectorConstants.MIN_SAFE_ANGLE_RADIAN).andThen(() -> pivotSafeDownPosEntry.setBoolean(false)));
    pivotDownPosTrigger.onTrue(
        endEffector.rotatePivot(() -> Constants.EndEffectorConstants.MIN_ANGLE_RADIAN).andThen(() -> pivotDownPosEntry.setBoolean(false)));
    pivotMiddlePosTrigger.onTrue(
        endEffector.rotatePivot(
            () ->
                (Constants.EndEffectorConstants.MIN_SAFE_ANGLE_RADIAN
                        + Constants.EndEffectorConstants.MAX_SAFE_ANGLE_RADIAN)
                    / 2).andThen(() -> pivotMiddlePosEntry.setBoolean(false)));
    pivotManualZeroTrigger.onTrue(endEffector.setPivotZero().andThen(() -> pivotManualZeroEntry.setBoolean(false)));
  }

  private void buildElevatorTab() {
    // Get the NetworkTable for the Elevator tab
    NetworkTable elevatorTable = NetworkTableInstance.getDefault().getTable("Elevator");

    // Create NetworkTableEntry instances for while-held functionality
    NetworkTableEntry elevatorUpEntry = elevatorTable.getEntry("Elevator Up (While Held)");
    NetworkTableEntry elevatorDownEntry = elevatorTable.getEntry("Elevator Down (While Held)");

    NetworkTableEntry elevatorL2Entry = elevatorTable.getEntry("Elevator L2 (When Pressed)");
    NetworkTableEntry elevatorL3Entry = elevatorTable.getEntry("Elevator L3 (When Pressed)");
    NetworkTableEntry elevatorL4Entry = elevatorTable.getEntry("Elevator L4 (When Pressed)");
    NetworkTableEntry elevatorDownPosEntry =
        elevatorTable.getEntry("Elevator Down Pos (When Pressed)");
    NetworkTableEntry elevatorManualZeroEntry =
        elevatorTable.getEntry("Zero the Elevator (When Pressed)");

    // Initialize entries with default values
    elevatorUpEntry.setBoolean(false);
    elevatorDownEntry.setBoolean(false);
    elevatorL2Entry.setBoolean(false);
    elevatorL3Entry.setBoolean(false);
    elevatorL4Entry.setBoolean(false);
    elevatorDownPosEntry.setBoolean(false);
    elevatorManualZeroEntry.setBoolean(false);

    // Create triggers based on the NetworkTableEntry values
    Trigger elevatorUpTrigger = new Trigger(() -> elevatorUpEntry.getBoolean(false));
    Trigger elevatorDownTrigger = new Trigger(() -> elevatorDownEntry.getBoolean(false));
    Trigger elevatorL2Trigger = new Trigger(() -> elevatorL2Entry.getBoolean(false));
    Trigger elevatorL3Trigger = new Trigger(() -> elevatorL3Entry.getBoolean(false));
    Trigger elevatorL4Trigger = new Trigger(() -> elevatorL4Entry.getBoolean(false));
    Trigger elevatorDownPosTrigger = new Trigger(() -> elevatorDownPosEntry.getBoolean(false));
    Trigger elevatorManualZeroTrigger =
        new Trigger(() -> elevatorManualZeroEntry.getBoolean(false));

    // Configure the while-held behavior
    elevatorUpTrigger.whileTrue(elevator.elevatorUP());
    elevatorUpTrigger.onFalse(elevator.elevatorSTOP());

    elevatorDownTrigger.whileTrue(elevator.elevatorDWN());
    elevatorDownTrigger.onFalse(elevator.elevatorSTOP());

    elevatorL2Trigger.onTrue(
        elevator.manualSetPosition(
            () -> Constants.SuperstructureConstants.L2_SCORE_ELEVATOR_HEIGHT_INCH).andThen(() -> elevatorL2Entry.setBoolean(false)));
    elevatorL3Trigger.onTrue(
        elevator.manualSetPosition(() -> SuperstructureState.L3_SCORE.getElevatorHeight()).andThen(() -> elevatorL3Entry.setBoolean(false)));
    elevatorL4Trigger.onTrue(
        elevator.manualSetPosition(() -> SuperstructureState.L4_SCORE.getElevatorHeight()).andThen(() -> elevatorL4Entry.setBoolean(false)));
    elevatorDownPosTrigger.onTrue(
        elevator.manualSetPosition(() -> SuperstructureState.STOW.getElevatorHeight()).andThen(() -> elevatorDownPosEntry.setBoolean(false)));
    elevatorManualZeroTrigger.onTrue(elevator.manualSetElevatorZero().andThen(() -> elevatorManualZeroEntry.setBoolean(false)));
  }

  private void buildSuperstructureTab() {
    // Get the NetworkTable for the Superstructure tab
    NetworkTable superstructureTable = NetworkTableInstance.getDefault().getTable("Superstructure");

    // Create NetworkTableEntry instances for each SuperstructureState
    NetworkTableEntry stowEntry = superstructureTable.getEntry("STOW");
    NetworkTableEntry stowCoralEntry = superstructureTable.getEntry("STOW_CORAL");
    NetworkTableEntry stowAlgaeEntry = superstructureTable.getEntry("STOW_ALGAE");
    NetworkTableEntry intakeCoralEntry = superstructureTable.getEntry("INTAKE_CORAL");
    NetworkTableEntry intakeCoralL1Entry = superstructureTable.getEntry("INTAKE_CORAL_L1");
    NetworkTableEntry feedEntry = superstructureTable.getEntry("FEED");
    NetworkTableEntry l1PivotEntry = superstructureTable.getEntry("L1_PIVOT");
    NetworkTableEntry l2AimEntry = superstructureTable.getEntry("L2_AIM");
    NetworkTableEntry l3AimEntry = superstructureTable.getEntry("L3_AIM");
    NetworkTableEntry l4AimEntry = superstructureTable.getEntry("L4_AIM");
    NetworkTableEntry l1ScoreEntry = superstructureTable.getEntry("L1_SCORE");
    NetworkTableEntry l2ScoreEntry = superstructureTable.getEntry("L2_SCORE");
    NetworkTableEntry l3ScoreEntry = superstructureTable.getEntry("L3_SCORE");
    NetworkTableEntry l4ScoreEntry = superstructureTable.getEntry("L4_SCORE");
    NetworkTableEntry algaeHighIntakeEntry = superstructureTable.getEntry("ALGAE_HIGH_INTAKE");
    NetworkTableEntry algaeLowIntakeEntry = superstructureTable.getEntry("ALGAE_LOW_INTAKE");
    NetworkTableEntry processorAimEntry = superstructureTable.getEntry("PROCESSOR_AIM");
    NetworkTableEntry bargeAimCenterEntry = superstructureTable.getEntry("BARGE_AIM_CENTER");
    NetworkTableEntry bargeAimForwardEntry = superstructureTable.getEntry("BARGE_AIM_FORWARD");
    NetworkTableEntry bargeAimBackwardEntry = superstructureTable.getEntry("BARGE_AIM_BACKWARD");

    // Initialize entries with default values
    stowEntry.setBoolean(false);
    stowCoralEntry.setBoolean(false);
    stowAlgaeEntry.setBoolean(false);
    intakeCoralEntry.setBoolean(false);
    intakeCoralL1Entry.setBoolean(false);
    feedEntry.setBoolean(false);
    l1PivotEntry.setBoolean(false);
    l2AimEntry.setBoolean(false);
    l3AimEntry.setBoolean(false);
    l4AimEntry.setBoolean(false);
    l1ScoreEntry.setBoolean(false);
    l2ScoreEntry.setBoolean(false);
    l3ScoreEntry.setBoolean(false);
    l4ScoreEntry.setBoolean(false);
    algaeHighIntakeEntry.setBoolean(false);
    algaeLowIntakeEntry.setBoolean(false);
    processorAimEntry.setBoolean(false);
    bargeAimCenterEntry.setBoolean(false);
    bargeAimForwardEntry.setBoolean(false);
    bargeAimBackwardEntry.setBoolean(false);

    // Create triggers for each button
    Trigger stowTrigger = new Trigger(() -> stowEntry.getBoolean(false));
    Trigger stowCoralTrigger = new Trigger(() -> stowCoralEntry.getBoolean(false));
    Trigger stowAlgaeTrigger = new Trigger(() -> stowAlgaeEntry.getBoolean(false));
    Trigger intakeCoralTrigger = new Trigger(() -> intakeCoralEntry.getBoolean(false));
    Trigger intakeCoralL1Trigger = new Trigger(() -> intakeCoralL1Entry.getBoolean(false));
    Trigger feedTrigger = new Trigger(() -> feedEntry.getBoolean(false));
    Trigger l1PivotTrigger = new Trigger(() -> l1PivotEntry.getBoolean(false));
    Trigger l2AimTrigger = new Trigger(() -> l2AimEntry.getBoolean(false));
    Trigger l3AimTrigger = new Trigger(() -> l3AimEntry.getBoolean(false));
    Trigger l4AimTrigger = new Trigger(() -> l4AimEntry.getBoolean(false));
    Trigger l1ScoreTrigger = new Trigger(() -> l1ScoreEntry.getBoolean(false));
    Trigger l2ScoreTrigger = new Trigger(() -> l2ScoreEntry.getBoolean(false));
    Trigger l3ScoreTrigger = new Trigger(() -> l3ScoreEntry.getBoolean(false));
    Trigger l4ScoreTrigger = new Trigger(() -> l4ScoreEntry.getBoolean(false));
    Trigger algaeHighIntakeTrigger = new Trigger(() -> algaeHighIntakeEntry.getBoolean(false));
    Trigger algaeLowIntakeTrigger = new Trigger(() -> algaeLowIntakeEntry.getBoolean(false));
    Trigger processorAimTrigger = new Trigger(() -> processorAimEntry.getBoolean(false));
    Trigger bargeAimCenterTrigger = new Trigger(() -> bargeAimCenterEntry.getBoolean(false));
    Trigger bargeAimForwardTrigger = new Trigger(() -> bargeAimForwardEntry.getBoolean(false));
    Trigger bargeAimBackwardTrigger = new Trigger(() -> bargeAimBackwardEntry.getBoolean(false));

    // Wire triggers to superstructure state commands
    stowTrigger.onTrue(
        superstructure
            .setStateCommand(SuperstructureState.STOW, "Set STOW")
            .andThen(() -> stowEntry.setBoolean(false)));
    stowCoralTrigger.onTrue(
        superstructure
            .setStateCommand(SuperstructureState.STOW_CORAL, "Set STOW_CORAL")
            .andThen(() -> stowCoralEntry.setBoolean(false)));
    stowAlgaeTrigger.onTrue(
        superstructure
            .setStateCommand(SuperstructureState.STOW_ALGAE, "Set STOW_ALGAE")
            .andThen(() -> stowAlgaeEntry.setBoolean(false)));
    intakeCoralTrigger.onTrue(
        superstructure
            .setStateCommand(SuperstructureState.INTAKE_CORAL, "Set INTAKE_CORAL")
            .andThen(() -> intakeCoralEntry.setBoolean(false)));
    intakeCoralL1Trigger.onTrue(
        superstructure
            .setStateCommand(SuperstructureState.INTAKE_CORAL_L1, "Set INTAKE_CORAL_L1")
            .andThen(() -> intakeCoralL1Entry.setBoolean(false)));
    feedTrigger.onTrue(
        superstructure
            .setStateCommand(SuperstructureState.FEED, "Set FEED")
            .andThen(() -> feedEntry.setBoolean(false)));
    l1PivotTrigger.onTrue(
        superstructure
            .setStateCommand(SuperstructureState.L1_PIVOT, "Set L1_PIVOT")
            .andThen(() -> l1PivotEntry.setBoolean(false)));
    l2AimTrigger.onTrue(
        superstructure
            .setStateCommand(SuperstructureState.L2_AIM, "Set L2_AIM")
            .andThen(() -> l2AimEntry.setBoolean(false)));
    l3AimTrigger.onTrue(
        superstructure
            .setStateCommand(SuperstructureState.L3_AIM, "Set L3_AIM")
            .andThen(() -> l3AimEntry.setBoolean(false)));
    l4AimTrigger.onTrue(
        superstructure
            .setStateCommand(SuperstructureState.L4_AIM, "Set L4_AIM")
            .andThen(() -> l4AimEntry.setBoolean(false)));
    l1ScoreTrigger.onTrue(
        superstructure
            .setStateCommand(SuperstructureState.L1_SCORE, "Set L1_SCORE")
            .andThen(() -> l1ScoreEntry.setBoolean(false)));
    l2ScoreTrigger.onTrue(
        superstructure
            .setStateCommand(SuperstructureState.L2_SCORE, "Set L2_SCORE")
            .andThen(() -> l2ScoreEntry.setBoolean(false)));
    l3ScoreTrigger.onTrue(
        superstructure
            .setStateCommand(SuperstructureState.L3_SCORE, "Set L3_SCORE")
            .andThen(() -> l3ScoreEntry.setBoolean(false)));
    l4ScoreTrigger.onTrue(
        superstructure
            .setStateCommand(SuperstructureState.L4_SCORE, "Set L4_SCORE")
            .andThen(() -> l4ScoreEntry.setBoolean(false)));
    algaeHighIntakeTrigger.onTrue(
        superstructure
            .setStateCommand(SuperstructureState.ALGAE_HIGH_INTAKE, "Set ALGAE_HIGH_INTAKE")
            .andThen(() -> algaeHighIntakeEntry.setBoolean(false)));
    algaeLowIntakeTrigger.onTrue(
        superstructure
            .setStateCommand(SuperstructureState.ALGAE_LOW_INTAKE, "Set ALGAE_LOW_INTAKE")
            .andThen(() -> algaeLowIntakeEntry.setBoolean(false)));
    processorAimTrigger.onTrue(
        superstructure
            .setStateCommand(SuperstructureState.PROCESSOR_AIM, "Set PROCESSOR_AIM")
            .andThen(() -> processorAimEntry.setBoolean(false)));
    bargeAimCenterTrigger.onTrue(
        superstructure
            .setStateCommand(SuperstructureState.BARGE_AIM_CENTER, "Set BARGE_AIM_CENTER")
            .andThen(() -> bargeAimCenterEntry.setBoolean(false)));
    bargeAimForwardTrigger.onTrue(
        superstructure
            .setStateCommand(SuperstructureState.BARGE_AIM_FORWARD, "Set BARGE_AIM_FORWARD")
            .andThen(() -> bargeAimForwardEntry.setBoolean(false)));
    bargeAimBackwardTrigger.onTrue(
        superstructure
            .setStateCommand(SuperstructureState.BARGE_AIM_BACKWARD, "Set BARGE_AIM_BACKWARD")
            .andThen(() -> bargeAimBackwardEntry.setBoolean(false)));

    // Add current state and target state monitoring
    NetworkTableEntry currentStateEntry = superstructureTable.getEntry("Current State");
    NetworkTableEntry targetStateEntry = superstructureTable.getEntry("Target State");

    // Update these entries periodically in the periodic method of RobotContainer
    // For now, we'll create them so they appear on the dashboard
    currentStateEntry.setString("Unknown");
    targetStateEntry.setString("Unknown");
  }

  private void buildDriveTab() {
    NetworkTable driveTable = NetworkTableInstance.getDefault().getTable("Drive");

    NetworkTableEntry driveFeedforwardEntry = driveTable.getEntry("Characterize Feedforward");
    NetworkTableEntry driveSlipCurrentEntry = driveTable.getEntry("Characterize Slip Current");
    NetworkTableEntry driveWheelRadiusEntry = driveTable.getEntry("Characterize Wheel Radius");
    NetworkTableEntry driveStopXEntry = driveTable.getEntry("Drive Stop X");
    NetworkTableEntry driveForwardEntry = driveTable.getEntry("Drive Forward");
    NetworkTableEntry driveClockwiseEntry = driveTable.getEntry("Drive Turn Clockwise");

    driveFeedforwardEntry.setBoolean(false);
    driveSlipCurrentEntry.setBoolean(false);
    driveWheelRadiusEntry.setBoolean(false);
    driveStopXEntry.setBoolean(false);
    driveForwardEntry.setBoolean(false);
    driveClockwiseEntry.setBoolean(false);

    Trigger driveFeedforwardTrigger = new Trigger(() -> driveFeedforwardEntry.getBoolean(false));
    Trigger driveSlipCurrentTrigger = new Trigger(() -> driveSlipCurrentEntry.getBoolean(false));
    Trigger driveWheelRadiusTrigger = new Trigger(() -> driveWheelRadiusEntry.getBoolean(false));
    Trigger driveStopXTrigger = new Trigger(() -> driveStopXEntry.getBoolean(false));
    Trigger driveForwardTrigger = new Trigger(() -> driveForwardEntry.getBoolean(false));
    Trigger driveClockwiseTrigger = new Trigger(() -> driveClockwiseEntry.getBoolean(false));

    driveFeedforwardTrigger.whileTrue(DriveCommands.feedforwardCharacterization(drive));
    driveSlipCurrentTrigger.whileTrue(DriveCommands.slipCurrentCharacterization(drive));
    driveWheelRadiusTrigger.whileTrue(DriveCommands.wheelRadiusCharacterization(drive));
    driveStopXTrigger.onTrue(Commands.runOnce(drive::stopWithX, drive).andThen(() -> driveStopXEntry.setBoolean(false)));
    driveForwardTrigger.whileTrue(
        Commands.run(() -> drive.runVelocity(new ChassisSpeeds(0.5, 0.0, 0.0))));
    driveClockwiseTrigger.whileTrue(
        Commands.run(() -> drive.runVelocity(new ChassisSpeeds(0.0, 0.0, 0.5))));
  }

  private void buildClimberTab() {
    NetworkTable climberTable = NetworkTableInstance.getDefault().getTable("Climber");

    NetworkTableEntry climberOutEntry = climberTable.getEntry("Climber Out (While Held)");
    NetworkTableEntry climberDeployEntry = climberTable.getEntry("Climber Deploy (When Pressed)");
    NetworkTableEntry climberClimbEntry = climberTable.getEntry("Climber Climb (When Pressed)");

    climberOutEntry.setBoolean(false);
    climberDeployEntry.setBoolean(false);
    climberClimbEntry.setBoolean(false);

    Trigger climberOutTrigger = new Trigger(() -> climberOutEntry.getBoolean(false));
    Trigger climberDeployTrigger = new Trigger(() -> climberDeployEntry.getBoolean(false));
    Trigger climberClimbTrigger = new Trigger(() -> climberClimbEntry.getBoolean(false));

    climberOutTrigger.whileTrue(climber.climbVoltOut());
    climberOutTrigger.onFalse(climber.climbSTOP());
    climberDeployTrigger.onTrue(climber.climbDeploy().andThen(() -> climberDeployEntry.setBoolean(false)));
    climberClimbTrigger.onTrue(climber.climbClimb().andThen(() -> climberClimbEntry.setBoolean(false)));
  }

  private void buildTestTab() {
    
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureXboxBindings() {

    // Lock to 0° when A button is held
    controller
        .a()
        .whileTrue(
            DriveCommands.driveAtAngle(
                drive,
                () -> -controller.getLeftY(),
                () -> -controller.getLeftX(),
                () -> Rotation2d.kZero));

    // Switch to X pattern when X button is pressed
    controller.x().onTrue(Commands.runOnce(drive::stopWithX, drive));

    // Reset gyro to 0° when B button is pressed
    controller
        .b()
        .onTrue(
            Commands.runOnce(
                    () ->
                        drive.setPose(
                            new Pose2d(drive.getPose().getTranslation(), Rotation2d.kZero)),
                    drive)
                .ignoringDisable(true));
  }

  private void configureStreamDeckBindings() {
    StreamDeckButtonConfig inactiveConfig =
        new StreamDeckButtonConfig(LedState.kCOOrange.toString(), LedState.kOff.toString(), "");
    StreamDeckButtonConfig activeConfig =
        new StreamDeckButtonConfig(LedState.kCOTeal.toString(), LedState.kWhite.toString(), "");

    StreamDeckButton swerveXButton =
        new StreamDeckButton(3, 7, "Swerve X")
            .withInactiveConfig(inactiveConfig)
            .withActiveConfig(activeConfig)
            .withInactiveText("X")
            .withActiveText("Swerve X");
    StreamDeckButton intakeInButton =
        new StreamDeckButton(1, 0, "Intake In")
            .withInactiveConfig(inactiveConfig)
            .withActiveConfig(activeConfig)
            .withText("INT In");
    StreamDeckButton intakeOutButton =
        new StreamDeckButton(1, 1, "Intake Out")
            .withInactiveConfig(inactiveConfig)
            .withActiveConfig(activeConfig)
            .withText("INT Out");
    StreamDeckButton intakeUpButton =
        new StreamDeckButton(0, 0, "Intake Up")
            .withInactiveConfig(inactiveConfig)
            .withActiveConfig(activeConfig)
            .withText("INT Up");
    StreamDeckButton intakeDownButton =
        new StreamDeckButton(0, 1, "Intake Down")
            .withInactiveConfig(inactiveConfig)
            .withActiveConfig(activeConfig)
            .withText("INT Down");
    StreamDeckButton intakeL1UpButton =
        new StreamDeckButton(3, 0, "Intake L1Up")
            .withInactiveConfig(inactiveConfig)
            .withActiveConfig(activeConfig)
            .withText("INT L1Up");
    StreamDeckButton intakeL1DownButton =
        new StreamDeckButton(3, 1, "Intake L1Down")
            .withInactiveConfig(inactiveConfig)
            .withActiveConfig(activeConfig)
            .withText("INT L1Down");
    StreamDeckButton feederInButton =
        new StreamDeckButton(2, 0, "Feeder In ")
            .withInactiveConfig(inactiveConfig)
            .withActiveConfig(activeConfig)
            .withText("FEED In");
    StreamDeckButton feederOutButton =
        new StreamDeckButton(2, 1, "Feeder Out")
            .withInactiveConfig(inactiveConfig)
            .withActiveConfig(activeConfig)
            .withText("FEED Out ");
    StreamDeckButton intakePosUpButton =
        new StreamDeckButton(0, 3, "Intake PosUp")
            .withInactiveConfig(inactiveConfig)
            .withActiveConfig(activeConfig)
            .withText("INT PosUp");
    StreamDeckButton intakePosDownButton =
        new StreamDeckButton(0, 4, "Intake PosDown")
            .withInactiveConfig(inactiveConfig)
            .withActiveConfig(activeConfig)
            .withText("INT PosDown");
    StreamDeckButton intakePosScoreButton =
        new StreamDeckButton(0, 5, "Intake PosScore")
            .withInactiveConfig(inactiveConfig)
            .withActiveConfig(activeConfig)
            .withText("INT PosScore");
    StreamDeckButton intakeZeroButton =
        new StreamDeckButton(1, 3, "Intake Zero")
            .withInactiveConfig(inactiveConfig)
            .withActiveConfig(activeConfig)
            .withText("INT Zero");
    // StreamDeckButton intakeStateButton =
    //     new StreamDeckButton(0, 0, "Intake State")
    //         .withInactiveConfig(inactiveConfig)
    //         .withActiveConfig(activeConfig)
    //         .withText("INT State");

    streamdeck.configureButtons(
        Set.of(
            swerveXButton,
            intakeInButton,
            intakeOutButton,
            intakeUpButton,
            intakeDownButton,
            intakeL1UpButton,
            intakeL1DownButton,
            feederInButton,
            feederOutButton,
            intakePosUpButton,
            intakePosDownButton,
            intakePosScoreButton,
            intakeZeroButton));

    streamdeck.button(swerveXButton).onTrue(Commands.runOnce(drive::stopWithX, drive));

    streamdeck.button(intakeInButton).whileTrue(intake.intakeFWD());
    streamdeck.button(intakeInButton).onFalse(intake.intakeSTOP());
    streamdeck.button(intakeOutButton).whileTrue(intake.intakeRVS());
    streamdeck.button(intakeOutButton).onFalse(intake.intakeSTOP());
    streamdeck.button(intakeUpButton).whileTrue(intake.pivotManualTestForward());
    streamdeck.button(intakeUpButton).onFalse(intake.pivotStop());
    streamdeck.button(intakeDownButton).whileTrue(intake.pivotManualTestReverse());
    streamdeck.button(intakeDownButton).onFalse(intake.pivotStop());
    streamdeck.button(intakeL1UpButton).whileTrue(intake.l1BarFWD());
    streamdeck.button(intakeL1UpButton).onFalse(intake.l1BarSTOP());
    streamdeck.button(intakeL1DownButton).whileTrue(intake.l1BarRVS());
    streamdeck.button(intakeL1DownButton).onFalse(intake.l1BarSTOP());
    streamdeck.button(feederInButton).whileTrue(intake.feederFWD());
    streamdeck.button(feederInButton).onFalse(intake.feederSTOP());
    streamdeck.button(feederOutButton).whileTrue(intake.feederRVS());
    streamdeck.button(feederOutButton).onFalse(intake.feederSTOP());
    streamdeck.button(intakePosUpButton).onTrue(intake.setPivotUp());
    streamdeck.button(intakePosDownButton).onTrue(intake.movePivotDown());
    streamdeck.button(intakePosScoreButton).onTrue(intake.setPivotScoring());
    streamdeck.button(intakeZeroButton).onTrue(intake.zeroPivotAtPivotUp());
  }

  private void configureArbitraryTriggers() {
    feeder.dejamTrigger.onTrue(intake.dejamFeeder());
    elevator.elevatorObjectTrigger.onTrue(elevator.dejamElevator());
    intake.rejectCoralTrigger().whileTrue(intake.rejectCoralCommand());
  }
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }

  public Command defaultElevatorCommand() {
    if (superstructure.getCurrentState() == SuperstructureState.NONE) {
      return Commands.none();
    } else {
      return elevator.moveToTargetPosition(
          () -> superstructure.getCurrentState().getElevatorHeight());
    }
  }

  public Command defaultEndEffectorCommand() {
    if (superstructure.getCurrentState() == SuperstructureState.NONE) {
      return Commands.none();
    } else {
      return endEffector.rotatePivot(() -> superstructure.getCurrentState().getEndEffectorRotation());
    }
  }
}
