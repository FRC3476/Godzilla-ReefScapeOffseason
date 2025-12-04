package frc.robot.commands.test;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.end_effector.Claw;
import frc.robot.subsystems.feeder.Feeder;
import frc.robot.subsystems.intake.Intake;

public class CleaningTest extends ParallelCommandGroup {

  public CleaningTest(Intake intake, Claw claw, Feeder feeder) {
    addCommands(
        new InstantCommand(() -> claw.setVoltage(() -> 1)),
        new InstantCommand(() -> intake.setRollerVoltage(1)),
        new InstantCommand(() -> feeder.setRollerVoltage(1)));
  }
}
