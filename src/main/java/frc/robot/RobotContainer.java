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
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.DriveCommands;
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
import frc.robot.subsystems.led.Led;
import frc.robot.subsystems.led.LedIO;
import frc.robot.subsystems.led.LedIOHardware;
import frc.robot.subsystems.led.LedState;
import frc.robot.subsystems.superstructure.CoralStateTracker;
import frc.robot.subsystems.superstructure.CoralStateTracker.CoralPosition;
import frc.robot.Constants.IntakeConstants.IntakeState;
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
  private final Led led;
  private final RobotState robotState;

  // Controller
  private final CommandXboxController controller = new CommandXboxController(0);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    robotState = new RobotState();
    
    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        drive =
            new Drive(
                new GyroIOPigeon2(),
                new ModuleIOTalonFX(TunerConstants.FrontLeft),
                new ModuleIOTalonFX(TunerConstants.FrontRight),
                new ModuleIOTalonFX(TunerConstants.BackLeft),
                new ModuleIOTalonFX(TunerConstants.BackRight));

        intake = new Intake(new IntakeIOReal());
        endEffector = new EndEffector(new EndEffectorIOReal());
        elevator = new Elevator(new ElevatorIOReal());
        led = new Led(new LedIOHardware(), robotState);
        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIOSim(TunerConstants.FrontLeft),
                new ModuleIOSim(TunerConstants.FrontRight),
                new ModuleIOSim(TunerConstants.BackLeft),
                new ModuleIOSim(TunerConstants.BackRight));

        intake = new Intake(new IntakeIOSim());
        endEffector = new EndEffector(new EndEffectorIOSim());
        elevator = new Elevator(new ElevatorIOSim());
        led = new Led(new LedIO() {
          @Override
          public LedState getCurrentState() {
            return LedState.kOff;
          }
          
          @Override
          public void writePixels(LedState state) {
            // No-op for SIM
          }
          
          @Override
          public void writePixels(LedState[] states) {
            // No-op for SIM
          }
        }, robotState);
        break;

      default:
        // Replayed robot, disable IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});

        intake = new Intake(new IntakeIO() {});
        endEffector = new EndEffector(new EndEffectorIO() {});
        elevator = new Elevator(new ElevatorIO() {});
        led = new Led(new LedIO() {
          @Override
          public LedState getCurrentState() {
            return LedState.kOff;
          }
          
          @Override
          public void writePixels(LedState state) {
            // No-op for default/replay
          }
          
          @Override
          public void writePixels(LedState[] states) {
            // No-op for default/replay
          }
        }, robotState);
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
        "Drive SysId (Quasistatic Forward)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Reverse)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));

    BuildIntakeTab();
    BuildEndEffectorTab();
    BuildElevatorTab();

    // Configure the button bindings
    configureButtonBindings();

    // Configure arbitrary triggers
    configureArbitraryTriggers();
    
    led.setDefaultCommand(createLedDefaultCommand());
    
    setupLedSpecialPatterns();
  }

  private void BuildIntakeTab() {
    ShuffleboardTab testTab = Shuffleboard.getTab("Intake");

    testTab.add("Intake Forward", intake.intakeFWD()).withPosition(0, 4).withSize(2, 1);
    testTab.add("Intake Reverse", intake.intakeRVS()).withPosition(2, 4).withSize(2, 1);
    testTab.add("Intake Stop", intake.intakeSTOP()).withPosition(4, 4).withSize(2, 1);
  }

  private void BuildEndEffectorTab() {
    ShuffleboardTab testTab = Shuffleboard.getTab("EndEffector");

    testTab.add("EndEffector Forward", endEffector.rollerFWD()).withPosition(0, 4).withSize(2, 1);
    testTab.add("EndEffector Reverse", endEffector.rollerRVS()).withPosition(2, 4).withSize(2, 1);
    testTab.add("EndEffector Stop", endEffector.rollerSTOP()).withPosition(4, 4).withSize(2, 1);
  }

  private void BuildElevatorTab() {
    ShuffleboardTab testTab = Shuffleboard.getTab("Elevator");

    // Create boolean entries for while-held functionality
    var elevatorUpHeld =
        testTab.add("Elevator Up (While Held)", false).withPosition(0, 5).withSize(2, 1).getEntry();

    var elevatorDownHeld =
        testTab
            .add("Elevator Down (While Held)", false)
            .withPosition(2, 5)
            .withSize(2, 1)
            .getEntry();

    // Create triggers based on the boolean entries
    Trigger elevatorUpTrigger = new Trigger(() -> elevatorUpHeld.getBoolean(false));
    Trigger elevatorDownTrigger = new Trigger(() -> elevatorDownHeld.getBoolean(false));

    // Configure the while-held behavior
    elevatorUpTrigger.whileTrue(elevator.elevatorUP());
    elevatorUpTrigger.onFalse(elevator.elevatorSTOP());

    elevatorDownTrigger.whileTrue(elevator.elevatorDWN());
    elevatorDownTrigger.onFalse(elevator.elevatorSTOP());
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

    // Default command for intake subsystem
    intake.setDefaultCommand(intake.intakeDefault());

    // Lock to 0° when A button is held
    controller
        .a()
        .whileTrue(
            DriveCommands.joystickDriveAtAngle(
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
   * Creates the default LED command that manages LED patterns based on robot state.
   */
  private Command createLedDefaultCommand() {
    return led.commandSolidColor(() -> {

      double batteryVoltage = RobotController.getBatteryVoltage();
      if (batteryVoltage < Constants.LEDConstants.kLowBatteryThresholdVolts) {
        return LedState.kLowBattery;
      }

      if (DriverStation.isDisabled()) {
        // Show alliance color when disabled
        if (DriverStation.getAlliance().isPresent()) {
          Alliance alliance = DriverStation.getAlliance().get();
          return alliance == Alliance.Red ? LedState.kRed : LedState.kBlue;
        }
        return LedState.kWhite; 
      }

      CoralPosition coralPosition = CoralStateTracker.getCurrentPosition();
      switch (coralPosition) {
        case AT_INTAKE:
          return LedState.kGreen; 
        case GOING_TO_FEEDER:
          return LedState.kYellow; 
        case AT_FEEDER:
          return LedState.kOrange; 
        case AT_FIRST_END_EFFECTOR:
          return LedState.kCyan; 
        case AT_SECOND_END_EFFECTOR:
          return LedState.kBlue; 
        case STAGED_IN_END_EFFECTOR:
          return LedState.kPurple; 
        case NONE:
        default:
          break; 
      }

      IntakeState intakeState = intake.getCurrentState();
      switch (intakeState) {
        case SCORING:
        case SCORING_PREP:
          return LedState.kCoralMode; 
        case REJECT_CORAL:
          return LedState.kRed; 
        case HAND_OFF:
          return LedState.kYellow; 
        case JAM_DETECTED:
          return LedState.kPink; 
        default:
          break; 
      }

      if (elevator.elevatorObjectTrigger.getAsBoolean()) {
        return LedState.kPink; 
      }

      if (endEffector.isCoralInEndeffector()) {
        return LedState.kCoralMode; 
      }
      if (endEffector.hasAlgae()) {
        return LedState.kAlgaeMode; 
      }

      if (DriverStation.isAutonomous()) {
        return LedState.kPurple; 
      }

      if (DriverStation.isTeleop()) {
        double time = Timer.getFPGATimestamp();
        double brightness = (Math.sin(time * 2.0) + 1.0) / 2.0; // https://www.desmos.com/calculator/qt2phfeona
        int scaledBrightness = (int) (brightness * 255);
        return new LedState(scaledBrightness, scaledBrightness, scaledBrightness);
      }

      // Fallback Default color
      return LedState.kCOOrange;
    }).withName("LED Default State Control");
  }

  /**
   * Sets up special LED pattern triggers for specific robot conditions, like a specific intake state, specific elevator state, CoralStateTracker, DriverStation, etc.
   * These patterns have higher priority and will override the default patterns.
   */
  private void setupLedSpecialPatterns() {
// These were just added so that some of the functions are locally used
    new Trigger(() -> intake.getCurrentState() == IntakeState.SCORING_PREP)
        .whileTrue(led.commandBlinkingState(LedState.kWhite, LedState.kCoralMode, 0.3));
        
    new Trigger(() -> elevator.elevatorObjectTrigger.getAsBoolean() || checkAnyJamCondition())
        .whileTrue(led.commandBlinkingState(LedState.kRed, LedState.kOff, 0.1));
        
 
    new Trigger(() -> CoralStateTracker.getCurrentPosition() == CoralPosition.STAGED_IN_END_EFFECTOR)
        .whileTrue(led.commandSolidPattern(getStagingLedPattern()));

    new Trigger(() -> DriverStation.isTeleop() && DriverStation.getMatchTime() <= 30.0 && DriverStation.getMatchTime() > 0.0)
        .whileTrue(led.commandSolidColor(() -> getAllianceEndgameColor()));
  }

  private boolean checkAnyJamCondition() {
    return intake.getCurrentState() == IntakeState.JAM_DETECTED;
  }

  /**
   * Gets the appropriate LED pattern based on elevator position for staging.
   */
  private LedState[] getStagingLedPattern() {
    double elevatorPosition = elevator.getCurrentPosition();
    
    if (elevatorPosition >= Constants.ElevatorConstants.ELEVATOR_L4_SETPOINT_INCH) { 
      return createFullLedArray(LedState.kPurple);
    } else if (elevatorPosition >= Constants.ElevatorConstants.ELEVATOR_L3_SETPOINT_INCH) { 
      return LedState.kL3StagingLeds;
    } else if (elevatorPosition >= Constants.ElevatorConstants.ELEVATOR_L2_SETPOINT_INCH) { 
      return LedState.kL2StagingLeds;
    } else {
      return LedState.kL2StagingLeds; 
    }
  }

  private LedState[] createFullLedArray(LedState color) {
    LedState[] fullArray = new LedState[Constants.LEDConstants.kMaxLEDCount];
    for (int i = 0; i < fullArray.length; i++) {
      fullArray[i] = color;
    }
    return fullArray;
  }

  private LedState getAllianceEndgameColor() {
    if (DriverStation.getAlliance().isPresent()) {
      Alliance alliance = DriverStation.getAlliance().get();

      return alliance == Alliance.Red ? LedState.kRed : LedState.kBlue;
    }
    return LedState.kYellow; 
  }
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }
}
