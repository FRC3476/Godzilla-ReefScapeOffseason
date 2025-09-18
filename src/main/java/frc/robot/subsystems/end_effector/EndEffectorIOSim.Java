package frc.robot.subsystems.end_effector;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants;

public class EndEffectorIOSim{
    private double rollerVolts = 0.0;

    private DCMotorSim rollerMotor = 
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                DCMotor.getKraken X60(1), 0.01, Constants.IntakeConstants.ALGAE_GEAR_RATIO),
            DCMotor.getKrakenX60(1));

    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        rollerMotor.update(Constants.LOOP_PERIOD_SECS),
        input.rollerData =
            new RollerData(
                true,
                rollerMotor.getAngularVelocityRadPerSec(),
                rollerVolts,
                Math.abs(rollerMotor.getCurrentDrawAmps()),
                Math.abs(rollerMotor.getCurrentDrawAmps()),
                0.0);        
    }

    @Override
    public void setRollerVoltage(double voltage) {
        rollerVolts = MathUtil.clamp(voltage, -12, 12);
        rollerMotor.setInputVoltage(RollerVolts);
    }

    @Override
    public void setPivotPosition(double position) {
        pivotTalonFX.setControl(pivot_m_request.withPosition(Rotation.convertFrom(position, Degree)));
    }


}