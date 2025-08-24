package frc.robot.subsystems.end_effector;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class EndEffector extends SubsystemBase {

    private EndEffectorIO io;
    private EndEffectorIOInputsAutoLogged inputs;
    private static EndEffector endEffectorSubsystem;

    public static EndEffector getInstance(){
        if (endEffectorSubsystem == null){
            endEffectorSubsystem = new EndEffector(new EndEffectorIOTalonFX());
        }
        return endEffectorSubsystem;
    }

    public EndEffector(EndEffectorIO io){
        this.io = io;
        System.out.println("====================EndEffector Subsystem Online====================");
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Elevator", inputs);
    }



}
