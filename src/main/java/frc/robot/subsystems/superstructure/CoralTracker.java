package frc.robot.subsystems.superstructure;

import java.security.InvalidKeyException;

import frc.robot.subsystems.end_effector.EndEffector;
import frc.robot.subsystems.feeder.Feeder;
import frc.robot.subsystems.intake.Intake;

public class CoralTracker {
    
    private Intake intake;
    private Feeder feeder;
    private EndEffector endEffector;

    private CoralPos coralPos;

    public enum CoralPos{
        NONE,
        INTAKE,
        FEEDER,
        ENDEFFECTOR
    }

    public CoralTracker(Intake intake, Feeder feeder, EndEffector end_effector){
        this.coralPos = CoralPos.NONE;
        this.intake = intake;
        this.feeder = feeder;
        this.endEffector = end_effector;
    }

    public void updateCoalPos(){
        if (intake.isCoralInIntake()) coralPos = CoralPos.INTAKE;
        else if (feeder.isCoralInFeeder()) coralPos = CoralPos.FEEDER;
        else if (endEffector.isCoralInEndeffector()) coralPos = CoralPos.ENDEFFECTOR;
    }

    public CoralPos getCoralPos(){
        return coralPos;
    }
    


}
