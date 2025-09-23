package frc.robot.commands.test;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.end_effector.EndEffector;
import frc.robot.subsystems.intake.Intake;

/**
 * Automatic Pre-Match Test that runs through all subsystem tests in sequence.
 */
public class AutomaticPreMatchTest extends SequentialCommandGroup {

  public AutomaticPreMatchTest(Intake intake, Elevator elevator, EndEffector endEffector, Drive drive) {
    addCommands(

        new PrintCommand("=== Starting Automatic Pre-Match Test ==="),
        new WaitCommand(1.0),
        
        new PrintCommand("Testing Intake System..."),
        new IntakeTest(intake),
        new WaitCommand(1.0),
        
        new PrintCommand("Testing Elevator & End Effector Systems..."),
        new ElevatorEndEffectorTest(elevator, endEffector),
        new WaitCommand(1.0),
        
        new PrintCommand("Testing Drivetrain System..."),
        new DrivetrainTest(drive),
        new WaitCommand(1.0),
        
        new PrintCommand("=== Automatic Pre-Match Test Complete ===")
    );
  }
}