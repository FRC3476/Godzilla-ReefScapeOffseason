package frc.robot.subsystems.end_effector;

import com.ctre.phoenix6.hardware.TalonFX;

import frc.robot.Constants.EndEffectorConstants;

public class EndEffectorIOTalonFX implements EndEffectorIO{
    
    private TalonFX pivotTalonFX;
    private TalonFX rollerTalonFX;

    public EndEffectorIOTalonFX(){
        pivotTalonFX = new TalonFX(EndEffectorConstants.endEffectorPivotID);
        rollerTalonFX = new TalonFX(EndEffectorConstants.endEffectorRollerID);

        pivotTalonFX.getConfigurator().apply(EndEffectorConstants.PIVOT_TALON_CONFIG);
        rollerTalonFX.getConfigurator().apply(EndEffectorConstants.ROLLER_TALON_CONFIG);
    }


}
