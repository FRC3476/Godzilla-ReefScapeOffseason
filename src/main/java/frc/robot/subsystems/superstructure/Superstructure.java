package frc.robot.subsystems.superstructure;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.end_effector.EndEffector;

public class Superstructure extends SubsystemBase {

    private EndEffector endEffector;
    private Elevator elevator;

    public Superstructure(EndEffector endEffector, Elevator elevator){
        this.endEffector = endEffector;
        this.elevator = elevator;
    }

    @Override
    public void periodic(){
        elevator.periodic();
        endEffector.periodic();
    }



}
