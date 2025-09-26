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
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.test.DrivetrainTest;
import frc.robot.generated.TunerConstants;
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
import frc.robot.subsystems.end_effector.EndEffector;
import frc.robot.subsystems.end_effector.EndEffectorIO;
import frc.robot.subsystems.end_effector.EndEffectorIOReal;
import frc.robot.subsystems.end_effector.EndEffectorIOSim;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeIO;
import frc.robot.subsystems.intake.IntakeIOReal;
import frc.robot.subsystems.intake.IntakeIOSim;
import frc.robot.subsystems.feeder.Feeder;
import frc.robot.subsystems.climb.Climber;
import frc.robot.subsystems.superstructure.Superstructure;
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
  private final Elevator elevator;
  private final Superstructure superstructure;
  private final Feeder feeder;
  private final Climber climber;

  // Controller
  private final CommandXboxController controller = new CommandXboxController(0);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        intake = new Intake(new IntakeIOReal());
        endEffector = new EndEffector(new EndEffectorIOReal());
        elevator = new Elevator(new ElevatorIOReal());
        superstructure = new Superstructure(elevator, endEffector);
        feeder = Feeder.getInstance();
        climber = Climber.getInstance();
        drive =
            new Drive(
                new GyroIOPigeon2(),
                new ModuleIOTalonFX(TunerConstants.FrontLeft),
                new ModuleIOTalonFX(TunerConstants.FrontRight),
                new ModuleIOTalonFX(TunerConstants.BackLeft),
                new ModuleIOTalonFX(TunerConstants.BackRight),
                superstructure);
        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        intake = new Intake(new IntakeIOSim());
        endEffector = new EndEffector(new EndEffectorIOSim());
        elevator = new Elevator(new ElevatorIOSim());
        superstructure = new Superstructure(elevator, endEffector);
        feeder = Feeder.getInstance();
        climber = Climber.getInstance();
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIOSim(TunerConstants.FrontLeft),
                new ModuleIOSim(TunerConstants.FrontRight),
                new ModuleIOSim(TunerConstants.BackLeft),
                new ModuleIOSim(TunerConstants.BackRight),
                superstructure);
        break;

      default:
        // Replayed robot, disable IO implementations
        intake = new Intake(new IntakeIO() {});
        endEffector = new EndEffector(new EndEffectorIO() {});
        elevator = new Elevator(new ElevatorIO() {});
        superstructure = new Superstructure(elevator, endEffector);
        feeder = Feeder.getInstance();
        climber = Climber.getInstance();
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                superstructure);
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

    BuildIntakeTab();
    BuildEndEffectorTab();
    BuildElevatorTab();
    BuildFeederTab();
    BuildClimberTab();
    BuildDriveTab();

    RegisterDefaultCommands();

    // Configure the button bindings
    configureButtonBindings();

    // Configure arbitrary triggers
    configureArbitraryTriggers();
  }

  private void BuildIntakeTab() {
    // Get the NetworkTable for the Intake tab
    NetworkTable intakeTable = NetworkTableInstance.getDefault().getTable("Intake");

    // Create NetworkTableEntry instances for while-held functionality
    NetworkTableEntry intakeForwardEntry = intakeTable.getEntry("Intake Forward (While Held)");
    NetworkTableEntry intakeReverseEntry = intakeTable.getEntry("Intake Reverse (While Held)");
    NetworkTableEntry pivotDownEntry = intakeTable.getEntry("Pivot Down");
    NetworkTableEntry engageL1Entry = intakeTable.getEntry("Engage L1 Blocker");
    NetworkTableEntry disengageL1Entry = intakeTable.getEntry("Disengage L1 Blocker");

    // Initialize entries with default values
    intakeForwardEntry.setBoolean(false);
    intakeReverseEntry.setBoolean(false);
    pivotDownEntry.setBoolean(false);
    engageL1Entry.setBoolean(false);
    disengageL1Entry.setBoolean(false);

    // Create triggers based on the NetworkTableEntry values
    Trigger intakeForwardTrigger = new Trigger(() -> intakeForwardEntry.getBoolean(false));
    Trigger intakeReverseTrigger = new Trigger(() -> intakeReverseEntry.getBoolean(false));
    Trigger pivotDownTrigger = new Trigger(() -> pivotDownEntry.getBoolean(false));
    Trigger engageL1Trigger = new Trigger(() -> engageL1Entry.getBoolean(false));
    Trigger disengageL1Trigger = new Trigger(() -> disengageL1Entry.getBoolean(false));

    // Configure the while-held behavior
    intakeForwardTrigger.whileTrue(intake.intakeFWD());
    intakeForwardTrigger.onFalse(intake.intakeSTOP());

    intakeReverseTrigger.whileTrue(intake.intakeRVS());
    intakeReverseTrigger.onFalse(intake.intakeSTOP());

    // Configure one-shot actions
    pivotDownTrigger.onTrue(intake.movePivotDown());
    engageL1Trigger.onTrue(intake.engageCoralL1());
    disengageL1Trigger.onTrue(intake.disengageCoralL1());
  }

  private void BuildEndEffectorTab() {
    // Get the NetworkTable for the EndEffector tab
    NetworkTable endEffectorTable = NetworkTableInstance.getDefault().getTable("EndEffector");

    // Create NetworkTableEntry instances for while-held functionality
    NetworkTableEntry endEffectorForwardEntry =
        endEffectorTable.getEntry("EndEffector Forward (While Held)");
    NetworkTableEntry endEffectorReverseEntry =
        endEffectorTable.getEntry("EndEffector Reverse (While Held)");

    // Initialize entries with default values
    endEffectorForwardEntry.setBoolean(false);
    endEffectorReverseEntry.setBoolean(false);

    // Create triggers based on the NetworkTableEntry values
    Trigger endEffectorForwardTrigger =
        new Trigger(() -> endEffectorForwardEntry.getBoolean(false));
    Trigger endEffectorReverseTrigger =
        new Trigger(() -> endEffectorReverseEntry.getBoolean(false));
    // Configure the while-held behavior
    endEffectorForwardTrigger.whileTrue(endEffector.rollerFWD());
    endEffectorForwardTrigger.onFalse(endEffector.rollerSTOP());

    endEffectorReverseTrigger.whileTrue(endEffector.rollerRVS());
    endEffectorReverseTrigger.onFalse(endEffector.rollerSTOP());
  }

  private void BuildElevatorTab() {
    // Get the NetworkTable for the Elevator tab
    NetworkTable elevatorTable = NetworkTableInstance.getDefault().getTable("Elevator");

    // Create NetworkTableEntry instances for while-held functionality
    NetworkTableEntry elevatorUpEntry = elevatorTable.getEntry("Elevator Up (While Held)");
    NetworkTableEntry elevatorDownEntry = elevatorTable.getEntry("Elevator Down (While Held)");

    // Initialize entries with default values
    elevatorUpEntry.setBoolean(false);
    elevatorDownEntry.setBoolean(false);

    // Create triggers based on the NetworkTableEntry values
    Trigger elevatorUpTrigger = new Trigger(() -> elevatorUpEntry.getBoolean(false));
    Trigger elevatorDownTrigger = new Trigger(() -> elevatorDownEntry.getBoolean(false));

    // Configure the while-held behavior
    elevatorUpTrigger.whileTrue(elevator.elevatorUP());
    elevatorUpTrigger.onFalse(elevator.elevatorSTOP());

    elevatorDownTrigger.whileTrue(elevator.elevatorDWN());
    elevatorDownTrigger.onFalse(elevator.elevatorSTOP());
  }

  private void RegisterDefaultCommands() {
    // elevator.setDefaultCommand(elevator.defaultElevatorCommand());
    // endEffector.setDefaultCommand(endEffector.defaultEndEffectorCommand());
    // intake.setDefaultCommand(intake.intakeDefault());
  }

  private void BuildDriveTab() {
    ShuffleboardTab testTab = Shuffleboard.getTab("Drive");

    testTab.add("Drivetrain Test", new DrivetrainTest(drive)).withPosition(0, 4).withSize(3, 1);

    testTab.add("Drive Stop", drive.run(drive::stop)).withPosition(3, 4).withSize(2, 1);

    testTab.add("Drive X-Lock", drive.run(drive::stopWithX)).withPosition(5, 4).withSize(2, 1);
  }

  private void BuildFeederTab() {
    NetworkTable feederTable = NetworkTableInstance.getDefault().getTable("Feeder");

    NetworkTableEntry feederForwardEntry = feederTable.getEntry("Feeder Forward (While Held)");
    NetworkTableEntry feederReverseEntry = feederTable.getEntry("Feeder Reverse (While Held)");

    feederForwardEntry.setBoolean(false);
    feederReverseEntry.setBoolean(false);

    Trigger feederForwardTrigger = new Trigger(() -> feederForwardEntry.getBoolean(false));
    Trigger feederReverseTrigger = new Trigger(() -> feederReverseEntry.getBoolean(false));

    feederForwardTrigger.whileTrue(Commands.run(() -> feeder.setRollerVoltage(12.0), feeder));
    feederForwardTrigger.onFalse(Commands.runOnce(() -> feeder.setRollerVoltage(0), feeder));

    feederReverseTrigger.whileTrue(Commands.run(() -> feeder.setRollerVoltageReversed(12.0), feeder));
    feederReverseTrigger.onFalse(Commands.runOnce(() -> feeder.setRollerVoltage(0), feeder));
  }

  private void BuildClimberTab() {
    NetworkTable climberTable = NetworkTableInstance.getDefault().getTable("Climber");

    NetworkTableEntry climberDeployEntry = climberTable.getEntry("Climber Deploy (While Held)");
    NetworkTableEntry climberStopEntry = climberTable.getEntry("Climber Stop");
    NetworkTableEntry climberManualUpEntry = climberTable.getEntry("Climber Manual Up (While Held)");
    NetworkTableEntry climberManualDownEntry = climberTable.getEntry("Climber Manual Down (While Held)");
    NetworkTableEntry climberHoldEntry = climberTable.getEntry("Climber Hold (While Held)");

    climberDeployEntry.setBoolean(false);
    climberStopEntry.setBoolean(false);
    climberManualUpEntry.setBoolean(false);
    climberManualDownEntry.setBoolean(false);
    climberHoldEntry.setBoolean(false);

    Trigger climberDeployTrigger = new Trigger(() -> climberDeployEntry.getBoolean(false));
    Trigger climberStopTrigger = new Trigger(() -> climberStopEntry.getBoolean(false));
    Trigger climberManualUpTrigger = new Trigger(() -> climberManualUpEntry.getBoolean(false));
    Trigger climberManualDownTrigger = new Trigger(() -> climberManualDownEntry.getBoolean(false));
    Trigger climberHoldTrigger = new Trigger(() -> climberHoldEntry.getBoolean(false));

    climberDeployTrigger.whileTrue(climber.climbDeploy());
    climberDeployTrigger.onFalse(climber.climbSTOP());

    climberStopTrigger.onTrue(climber.climbSTOP());

    climberManualUpTrigger.whileTrue(climber.climberManualUp());
    climberManualUpTrigger.onFalse(climber.climberHold());

    climberManualDownTrigger.whileTrue(climber.climberManualDown());
    climberManualDownTrigger.onFalse(climber.climberHold());

    climberHoldTrigger.whileTrue(climber.climberHold());
    climberHoldTrigger.onFalse(climber.climbSTOP());
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Default command, normal field-relative drive
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -controller.getLeftY(),
            () -> -controller.getLeftX(),
            () -> -controller.getRightX()));

    // Lock to 0° when A button is held
    controller
        .a()
        .whileTrue(
            DriveCommands.driveAtAngle(
                drive,
                () -> -controller.getLeftY(),
                () -> -controller.getLeftX(),
                () -> new Rotation2d()));

    // Switch to X pattern when X button is pressed
    controller.x().onTrue(Commands.runOnce(drive::stopWithX, drive));

    // Reset gyro to 0° when B button is pressed
    controller
        .b()
        .onTrue(
            Commands.runOnce(
                    () ->
                        drive.setPose(
                            new Pose2d(drive.getPose().getTranslation(), new Rotation2d())),
                    drive)
                .ignoringDisable(true));
  }

  private void configureArbitraryTriggers() {
    intake.feederJamTrigger.onTrue(intake.dejamFeeder());
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
    return elevator.moveToTargetPosition(
        () -> superstructure.getCurrentState().getElevatorHeight());
  }

  public Command defaultEndEffectorCommand() {
    return endEffector.rotatePivot(() -> superstructure.getCurrentState().getEndEffectorRotation());
  }
}
