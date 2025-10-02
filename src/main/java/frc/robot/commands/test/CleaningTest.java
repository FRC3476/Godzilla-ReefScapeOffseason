package frc.robot.commands.test;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.end_effector.Claw;
import frc.robot.subsystems.feeder.Feeder;
import frc.robot.subsystems.intake.Intake;

public class CleaningTest extends Command{

    public CleaningTest(Intake intake, Claw claw, Feeder feeder) {
        claw.setRollerVoltage(1);
        intake.setRollerVoltage(1);
        feeder.setRollerVoltage(1);
        intake.intakeFWD();
        intake.feederFWD();
        claw.rollerFWD();
    }

}
