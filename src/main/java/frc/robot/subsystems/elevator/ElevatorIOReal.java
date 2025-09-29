package frc.robot.subsystems.elevator;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.PhysicalConstants;
import frc.robot.util.MotorStallDetection;
import frc.robot.util.PhoenixUtil;

public class ElevatorIOReal implements ElevatorIO {

  protected TalonFX rightTalon;
  protected TalonFX leftTalon;
  protected TalonFX extraTalon;

  private MotionMagicVoltage m_request =
      new MotionMagicVoltage(PhysicalConstants.ABSOLUTE_ZERO).withEnableFOC(true);
  private VoltageOut m_VoltageOut =
      new VoltageOut(PhysicalConstants.ABSOLUTE_ZERO).withEnableFOC(true);

  // =====Logged Values=====
  StatusSignal<Angle> rightPosition;
  StatusSignal<Voltage> rightAppliedVolts;
  StatusSignal<Current> rightTorqueCurrentAmps;
  StatusSignal<Current> rightSupplyCurrentAmps;
  StatusSignal<Temperature> rightTempCelsius;
  StatusSignal<Double> rightSetPosition;

  StatusSignal<Angle> leftPosition;
  StatusSignal<Voltage> leftAppliedVolts;
  StatusSignal<Current> leftTorqueCurrentAmps;
  StatusSignal<Current> leftSupplyCurrentAmps;
  StatusSignal<Temperature> leftTempCelsius;
  StatusSignal<Double> leftSetPosition;

  StatusSignal<Angle> extraPosition;
  StatusSignal<Voltage> extraAppliedVolts;
  StatusSignal<Current> extraTorqueCurrentAmps;
  StatusSignal<Current> extraSupplyCurrentAmps;
  StatusSignal<Temperature> extraTempCelsius;
  StatusSignal<Double> extraSetPosition;

  private final BaseStatusSignal[] signals;

  public ElevatorIOReal() {
    rightTalon = new TalonFX(ElevatorConstants.elevatorRightID, Constants.misc_canivore);
    leftTalon = new TalonFX(ElevatorConstants.elevatorLeftID, Constants.misc_canivore);
    extraTalon = new TalonFX(ElevatorConstants.elevatorExtraID, Constants.misc_canivore);

    PhoenixUtil.tryUntilOk(
        5, () -> rightTalon.getConfigurator().apply(ElevatorConstants.elevatorRightTalon));
    leftTalon.setControl(new Follower(ElevatorConstants.elevatorRightID, true));
    extraTalon.setControl(new Follower(ElevatorConstants.elevatorRightID, true));

    rightPosition = rightTalon.getPosition();
    rightAppliedVolts = rightTalon.getMotorVoltage();
    rightTorqueCurrentAmps = rightTalon.getTorqueCurrent();
    rightSupplyCurrentAmps = rightTalon.getSupplyCurrent();
    rightTempCelsius = rightTalon.getDeviceTemp();
    rightSetPosition = rightTalon.getClosedLoopReference();

    leftPosition = leftTalon.getPosition();
    leftAppliedVolts = leftTalon.getMotorVoltage();
    leftTorqueCurrentAmps = leftTalon.getTorqueCurrent();
    leftSupplyCurrentAmps = leftTalon.getSupplyCurrent();
    leftTempCelsius = leftTalon.getDeviceTemp();
    leftSetPosition = leftTalon.getClosedLoopReference();

    extraPosition = extraTalon.getPosition();
    extraAppliedVolts = extraTalon.getMotorVoltage();
    extraTorqueCurrentAmps = extraTalon.getTorqueCurrent();
    extraSupplyCurrentAmps = extraTalon.getSupplyCurrent();
    extraTempCelsius = extraTalon.getDeviceTemp();
    extraSetPosition = extraTalon.getClosedLoopReference();

    signals =
        new BaseStatusSignal[] {
          rightPosition,
          rightAppliedVolts,
          rightTorqueCurrentAmps,
          rightSupplyCurrentAmps,
          rightTempCelsius,
          rightSetPosition,
          leftPosition,
          leftAppliedVolts,
          leftTorqueCurrentAmps,
          leftSupplyCurrentAmps,
          leftTempCelsius,
          leftSetPosition,
          extraPosition,
          extraAppliedVolts,
          extraTorqueCurrentAmps,
          extraSupplyCurrentAmps,
          extraTempCelsius,
          extraSetPosition
        };

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0,
        rightPosition,
        rightAppliedVolts,
        rightTorqueCurrentAmps,
        rightSupplyCurrentAmps,
        rightTempCelsius,
        rightSetPosition,
        leftPosition,
        leftAppliedVolts,
        leftTorqueCurrentAmps,
        leftSupplyCurrentAmps,
        leftTempCelsius,
        leftSetPosition,
        extraPosition,
        extraAppliedVolts,
        extraTorqueCurrentAmps,
        extraSupplyCurrentAmps,
        extraTempCelsius,
        extraSetPosition);
    ParentDevice.optimizeBusUtilizationForAll(rightTalon, leftTalon, extraTalon);
    PhoenixUtil.registerSignals(
        false,
        rightPosition,
        rightAppliedVolts,
        rightTorqueCurrentAmps,
        rightSupplyCurrentAmps,
        rightTempCelsius,
        rightSetPosition,
        leftPosition,
        leftAppliedVolts,
        leftTorqueCurrentAmps,
        leftSupplyCurrentAmps,
        leftTempCelsius,
        leftSetPosition,
        extraPosition,
        extraAppliedVolts,
        extraTorqueCurrentAmps,
        extraSupplyCurrentAmps,
        extraTempCelsius,
        extraSetPosition);
  }

  public void updateInputs(ElevatorIOInputs inputs) {
    BaseStatusSignal.refreshAll(signals);

    inputs.rightMotorData =
        new MotorData(
            BaseStatusSignal.isAllGood(
                rightPosition,
                rightAppliedVolts,
                rightTorqueCurrentAmps,
                rightSupplyCurrentAmps,
                rightTempCelsius,
                rightSetPosition),
            rightPosition.getValueAsDouble(),
            rightAppliedVolts.getValueAsDouble(),
            rightTorqueCurrentAmps.getValueAsDouble(),
            rightSupplyCurrentAmps.getValueAsDouble(),
            rightTempCelsius.getValueAsDouble(),
            rightSetPosition.getValueAsDouble());

    inputs.leftMotorData =
        new MotorData(
            BaseStatusSignal.isAllGood(
                leftPosition,
                leftAppliedVolts,
                leftTorqueCurrentAmps,
                leftSupplyCurrentAmps,
                leftTempCelsius,
                leftSetPosition),
            leftPosition.getValueAsDouble(),
            leftAppliedVolts.getValueAsDouble(),
            leftTorqueCurrentAmps.getValueAsDouble(),
            leftSupplyCurrentAmps.getValueAsDouble(),
            leftTempCelsius.getValueAsDouble(),
            leftSetPosition.getValueAsDouble());

    inputs.extraMotorData =
        new MotorData(
            BaseStatusSignal.isAllGood(
                extraPosition,
                extraAppliedVolts,
                extraTorqueCurrentAmps,
                extraSupplyCurrentAmps,
                extraTempCelsius,
                extraSetPosition),
            extraPosition.getValueAsDouble(),
            extraAppliedVolts.getValueAsDouble(),
            extraTorqueCurrentAmps.getValueAsDouble(),
            extraSupplyCurrentAmps.getValueAsDouble(),
            extraTempCelsius.getValueAsDouble(),
            extraSetPosition.getValueAsDouble());
  }

  @Override
  public void setElevatorVoltage(double voltage) {
    rightTalon.setControl(m_VoltageOut.withOutput(voltage));
  }

  @Override
  public void setElevatorTargetPosition(double position) {
    rightTalon.setControl(m_request.withPosition(position));
  }

  @Override
  public void setElevatorZero() {
    rightTalon.setPosition(0.0);
    leftTalon.setPosition(0.0);
    extraTalon.setPosition(0.0);
  }

  public void stop() {
    rightTalon.stopMotor();
  }

  @Override
  public boolean checkMotorsStalled() {
    return MotorStallDetection.isMotorStalled(
            rightTalon, ElevatorConstants.STALLED_CURRENT, ElevatorConstants.STALLED_RPS)
        || MotorStallDetection.isMotorStalled(
            leftTalon, ElevatorConstants.STALLED_CURRENT, ElevatorConstants.STALLED_RPS);
  }

  @Override
  public void updateElevatorPIDFF(
      double kP,
      double kI,
      double kD,
      double kG,
      double kS,
      double velo,
      double accel,
      double jerk) {
    var rightConfig = new TalonFXConfiguration();
    rightTalon.getConfigurator().refresh(rightConfig);
    rightConfig.Slot0.kP = kP;
    rightConfig.Slot0.kI = kI;
    rightConfig.Slot0.kD = kD;
    rightConfig.Slot0.kG = kG;
    rightConfig.Slot0.kS = kS;
    rightConfig.MotionMagic.MotionMagicCruiseVelocity = velo;
    rightConfig.MotionMagic.MotionMagicAcceleration = accel;
    rightConfig.MotionMagic.MotionMagicJerk = jerk;
    PhoenixUtil.tryUntilOk(5, () -> rightTalon.getConfigurator().apply(rightConfig, 0.050));
  }
}
