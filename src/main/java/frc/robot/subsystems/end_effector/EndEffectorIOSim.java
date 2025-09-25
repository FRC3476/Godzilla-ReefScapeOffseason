package frc.robot.subsystems.end_effector;

import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants;
import frc.robot.Constants.EndEffectorConstants;

public class EndEffectorIOSim extends EndEffectorIOReal {
  
  protected DCMotorSim pivotSim;
  protected DCMotorSim rollerSim;
  protected Notifier simNotifier;


  private final TalonFXSimState pivotSimState;
  private final TalonFXSimState rollerSimState;


  protected double lastUpdateTimestamp = 0.0;

  public EndEffectorIOSim() {
    super();

    pivotSim = 
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                DCMotor.getKrakenX60(1),
                0.01,
                1.0 / EndEffectorConstants.PIVOT_GEAR_RATIO
            ), DCMotor.getKrakenX60(1)
        );
    rollerSim = 
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                DCMotor.getKrakenX60(1),
                0.01,
                1.0 / EndEffectorConstants.ALGAE_GEAR_RATIO
            ), DCMotor.getKrakenX60(1)
        );
    
    pivotTalonFX.getSimState().Orientation = ChassisReference.Clockwise_Positive;
    rollerTalonFX.getSimState().Orientation = ChassisReference.Clockwise_Positive;

    pivotSimState = pivotTalonFX.getSimState();
    rollerSimState = rollerTalonFX.getSimState();

    simNotifier =
        new Notifier(
              () -> {
                updateSimState();
              }
        );
    simNotifier.startPeriodic(0.005);
  }

  private void updateSimState() {
    pivotSimState.setSupplyVoltage(RobotController.getBatteryVoltage());
    rollerSimState.setSupplyVoltage(RobotController.getBatteryVoltage());
  }

  /*
  private double rollerVolts = 0.0;

  private DCMotorSim rollerMotor =
      new DCMotorSim(
          LinearSystemId.createDCMotorSystem(
              DCMotor.getKrakenX60(1), 0.01, Constants.EndEffectorConstants.ALGAE_GEAR_RATIO),
          DCMotor.getKrakenX60(1));

  @Override
  public void updateInputs(EndEffectorIOInputs inputs) {
    rollerMotor.update(Constants.LOOP_PERIOD_SECS);
    inputs.rollerData =
        new EE_RollerData(
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
    rollerMotor.setInputVoltage(rollerVolts);
  }

  @Override
  public void setRollerVelocity(double velocity) {}

  @Override
  public void setPivotTargetPosition(double position) {}*/
}
