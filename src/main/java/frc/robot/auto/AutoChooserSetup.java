package frc.robot.auto;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.ContinuousPathTestCommand;
import frc.robot.subsystems.drive.DriveSubsystem;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class AutoChooserSetup {

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  public AutoChooserSetup(DriveSubsystem drive) {

    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    // Continuous Path Following Test Commands (Team 2056 approach)
    autoChooser.addOption(
        "Continuous Path: Forward Test", ContinuousPathTestCommand.forwardPathTest(drive));
    autoChooser.addOption(
        "Continuous Path: Square Test", ContinuousPathTestCommand.squareTest(drive));
    autoChooser.addOption(
        "Continuous Path: Zigzag Test", ContinuousPathTestCommand.zigzagTest(drive));
    autoChooser.addOption(
        "Continuous Path: Variable Switching Test",
        ContinuousPathTestCommand.variableSwitchingDistanceTest(drive));
    autoChooser.addOption(
        "Continuous Path: Right Raw Test (RightRaw1 -> RightRaw2)",
        ContinuousPathTestCommand.rightRawPathTest(drive));
    autoChooser.addOption(
        "Continuous Path: Comprehensive Test", ContinuousPathTestCommand.comprehensiveTest(drive));
    autoChooser.addOption(
        "Continuous Path: Comparison Test (vs Point-to-Point)",
        ContinuousPathTestCommand.comparisonTest(drive));

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
