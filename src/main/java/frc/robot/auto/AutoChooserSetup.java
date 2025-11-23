package frc.robot.auto;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.commands.ContinuousPathTestCommand;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class AutoChooserSetup {

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  public AutoChooserSetup(RobotContainer container) {

    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    // Continuous Path Following Test Commands (Team 2056 approach)
    autoChooser.addOption(
        "Continuous Path: Forward Test", ContinuousPathTestCommand.forwardPathTest(container));
    autoChooser.addOption(
        "Continuous Path: Square Test", ContinuousPathTestCommand.squareTest(container));
    autoChooser.addOption(
        "Continuous Path: Zigzag Test", ContinuousPathTestCommand.zigzagTest(container));
    autoChooser.addOption(
        "Continuous Path: Variable Switching Test",
        ContinuousPathTestCommand.variableSwitchingDistanceTest(container));
    autoChooser.addOption(
        "Continuous Path: Right Raw Test (RightRaw1 -> RightRaw2)",
        ContinuousPathTestCommand.rightRawPathTest(container));
    autoChooser.addOption(
        "Continuous Path: Comprehensive Test",
        ContinuousPathTestCommand.comprehensiveTest(container));
    autoChooser.addOption(
        "Continuous Path: Comparison Test (vs Point-to-Point)",
        ContinuousPathTestCommand.comparisonTest(container));

    // Set up SysId routines
    // autoChooser.addOption(
    //     "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
    // autoChooser.addOption(
    //     "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
    // autoChooser.addOption(
    //     "Drive Slip Current Characterization (Wall Test)",
    //     DriveCommands.slipCurrentCharacterization(drive));
    // autoChooser.addOption(
    //     "Drive SysId (Quasistatic Forward)",
    //     drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    // autoChooser.addOption(
    //     "Drive SysId (Quasistatic Reverse)",
    //     drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    // autoChooser.addOption(
    //     "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    // autoChooser.addOption(
    //     "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));

    // autoChooser.addOption("Drivetrain Test", new DrivetrainTest(drive));
  }

  public Command getAutonomousCommand() {
    return autoChooser.get();
  }
}
