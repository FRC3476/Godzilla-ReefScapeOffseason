package frc.robot.commands.test;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.intake.Intake;

public class IntakeTest extends SequentialCommandGroup {
  /** Creates a new IntakeTest. */
  public IntakeTest(Intake intake) {
    addCommands(
        intake.intakeFWD(),
        new WaitCommand(2.0),
        intake.intakeSTOP(),
        new WaitCommand(0.5),
        intake.intakeRVS(),
        new WaitCommand(2.0),
        intake.intakeSTOP(),
        new WaitCommand(0.5),
        intake.setPivotDown(),
        new WaitCommand(1.0),
        intake.engageCoralL1(),
        new WaitCommand(1.0),
        intake.disengageCoralL1(),
        new WaitCommand(1.0));
  }
}
