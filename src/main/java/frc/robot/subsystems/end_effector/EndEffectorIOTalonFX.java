package frc.robot.subsystems.end_effector;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;

import frc.robot.Constants.EndEffectorConstants;
import frc.robot.Constants.PhysicalConstants;

public class EndEffectorIOTalonFX implements EndEffectorIO{
    
    private TalonFX pivotTalonFX;
    private TalonFX rollerTalonFX;


    private MotionMagicVoltage pivot_m_request =
      new MotionMagicVoltage(PhysicalConstants.ABSOLUTE_ZERO).withEnableFOC(true);

  // =====Logged Values=====
  StatusSignal<Angle> pivotPosition;
  StatusSignal<Voltage> pivotAppliedVolts;
  StatusSignal<Current> pivotTorqueCurrentAmps;
  StatusSignal<Current> pivotSupplyCurrentAmps;
  StatusSignal<Temperature> pivotTempCelsius;

  StatusSignal<Angle> rollerPosition;
  StatusSignal<Voltage> rollerAppliedVolts;
  StatusSignal<Current> rollerTorqueCurrentAmps;
  StatusSignal<Current> rollerSupplyCurrentAmps;
  StatusSignal<Temperature> rollerTempCelsius;


    public EndEffectorIOTalonFX(){
        pivotTalonFX = new TalonFX(EndEffectorConstants.endEffectorPivotID);
        rollerTalonFX = new TalonFX(EndEffectorConstants.endEffectorRollerID);

        pivotTalonFX.getConfigurator().apply(EndEffectorConstants.PIVOT_TALON_CONFIG);
        rollerTalonFX.getConfigurator().apply(EndEffectorConstants.ROLLER_TALON_CONFIG);
    
        pivotPosition = pivotTalonFX.getPosition();
        pivotAppliedVolts = pivotTalonFX.getMotorVoltage();
        pivotTorqueCurrentAmps = pivotTalonFX.getTorqueCurrent();
        pivotSupplyCurrentAmps = pivotTalonFX.getSupplyCurrent();
        pivotTempCelsius = pivotTalonFX.getDeviceTemp();

        rollerPosition = rollerTalonFX.getPosition();
        rollerAppliedVolts = rollerTalonFX.getMotorVoltage();
        rollerTorqueCurrentAmps = rollerTalonFX.getTorqueCurrent();
        rollerSupplyCurrentAmps = rollerTalonFX.getSupplyCurrent();
        rollerTempCelsius = rollerTalonFX.getDeviceTemp();

       
    
    }

    public void updateInputs (EndEffectorIOInputs inputs){
    }


}
