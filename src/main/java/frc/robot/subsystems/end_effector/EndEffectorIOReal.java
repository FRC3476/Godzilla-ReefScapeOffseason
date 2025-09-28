package frc.robot.subsystems.end_effector;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.ControlModeValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants;
import frc.robot.Constants.EndEffectorConstants;
import frc.robot.Constants.PhysicalConstants;
import frc.robot.util.PhoenixUtil;

public class EndEffectorIOReal implements EndEffectorIO {

  private TalonFX pivotTalonFX;
  private CANcoder pivotCancoder;

  private MotionMagicVoltage pivot_m_request =
      new MotionMagicVoltage(PhysicalConstants.ABSOLUTE_ZERO).withEnableFOC(true);

  private VoltageOut pivotVoltageRequest =
      new VoltageOut(PhysicalConstants.ABSOLUTE_ZERO).withEnableFOC(true);

  private final BaseStatusSignal[] signals;

  // =====Logged Values=====
  StatusSignal<Angle> pivotPosition;
  StatusSignal<Voltage> pivotAppliedVolts;
  StatusSignal<Current> pivotTorqueCurrentAmps;
  StatusSignal<Current> pivotSupplyCurrentAmps;
  StatusSignal<Temperature> pivotTempCelsius;
  StatusSignal<Double> pivotSetpoint;
  StatusSignal<ControlModeValue> pivotControlMode;

  public EndEffectorIOReal() {
    pivotTalonFX = new TalonFX(EndEffectorConstants.pivotID, Constants.misc_canivore);
    PhoenixUtil.tryUntilOk(
        5, () -> pivotTalonFX.getConfigurator().apply(EndEffectorConstants.PIVOT_TALON_CONFIG));

    pivotCancoder = new CANcoder(EndEffectorConstants.PIVOT_CANCODER_ID, Constants.misc_canivore);
    PhoenixUtil.tryUntilOk(
        5, () -> pivotCancoder.getConfigurator().apply(EndEffectorConstants.PIVOT_CANCODER_CONFIG));

    pivotPosition = pivotTalonFX.getPosition();
    pivotAppliedVolts = pivotTalonFX.getMotorVoltage();
    pivotTorqueCurrentAmps = pivotTalonFX.getTorqueCurrent();
    pivotSupplyCurrentAmps = pivotTalonFX.getSupplyCurrent();
    pivotTempCelsius = pivotTalonFX.getDeviceTemp();
    pivotSetpoint = pivotTalonFX.getClosedLoopReference();
    pivotControlMode = pivotTalonFX.getControlMode();

    signals =
        new BaseStatusSignal[] {
          pivotPosition,
          pivotAppliedVolts,
          pivotTorqueCurrentAmps,
          pivotSupplyCurrentAmps,
          pivotTempCelsius,
          pivotSetpoint,
          pivotControlMode
        };

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0,
        pivotPosition,
        pivotAppliedVolts,
        pivotTorqueCurrentAmps,
        pivotSupplyCurrentAmps,
        pivotTempCelsius,
        pivotSetpoint,
        pivotControlMode);
    ParentDevice.optimizeBusUtilizationForAll(pivotTalonFX);
    PhoenixUtil.registerSignals(
        true,
        pivotPosition,
        pivotAppliedVolts,
        pivotTorqueCurrentAmps,
        pivotSupplyCurrentAmps,
        pivotTempCelsius,
        pivotSetpoint,
        pivotControlMode);
  }

  @Override
  public void updateInputs(EndEffectorIOInputs inputs) {
    BaseStatusSignal.refreshAll(signals);
    inputs.pivotData =
        new EE_PivotData(
            BaseStatusSignal.isAllGood(
                pivotPosition,
                pivotAppliedVolts,
                pivotTorqueCurrentAmps,
                pivotSupplyCurrentAmps,
                pivotTempCelsius,
                pivotSetpoint,
                pivotControlMode),
            pivotPosition.getValueAsDouble(),
            pivotAppliedVolts.getValueAsDouble(),
            pivotTorqueCurrentAmps.getValueAsDouble(),
            pivotSupplyCurrentAmps.getValueAsDouble(),
            pivotTempCelsius.getValueAsDouble(),
            pivotSetpoint.getValueAsDouble(),
            pivotControlMode.getValue().toString());
  }

  @Override
  public void setPivotVoltage(double voltage) {
    pivotTalonFX.setControl(pivotVoltageRequest.withOutput(voltage));
  }

  @Override
  public void setPivotPosition(double position) {
    pivotTalonFX.setControl(pivot_m_request.withPosition(position / 2 / Math.PI));
  }

  @Override
  public void updatePivotPIDFF(
      double kP,
      double kI,
      double kD,
      double kG,
      double kS,
      double velo,
      double accel,
      double jerk) {
    var pivotConfig = new TalonFXConfiguration();
    pivotTalonFX.getConfigurator().refresh(pivotConfig);
    pivotConfig.Slot0.kP = kP;
    pivotConfig.Slot0.kI = kI;
    pivotConfig.Slot0.kD = kD;
    pivotConfig.Slot0.kG = kG;
    pivotConfig.Slot0.kS = kS;
    pivotConfig.MotionMagic.MotionMagicCruiseVelocity = velo;
    pivotConfig.MotionMagic.MotionMagicAcceleration = accel;
    pivotConfig.MotionMagic.MotionMagicJerk = jerk;
    PhoenixUtil.tryUntilOk(5, () -> pivotTalonFX.getConfigurator().apply(pivotConfig, 0.050));
  }

  @Override
  public void setPivotZero() {
    pivotCancoder.setPosition(0);
    // pivotTalonFX.setPosition(0.0);
  }
}
