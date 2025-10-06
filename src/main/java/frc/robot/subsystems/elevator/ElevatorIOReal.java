package frc.robot.subsystems.elevator;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.PhysicalConstants;
import frc.robot.util.MotorStallDetection;
import frc.robot.util.PhoenixUtil;
import java.util.function.DoubleSupplier;

public class ElevatorIOReal implements ElevatorIO {

  protected TalonFX rightTalon;
  protected TalonFX leftTalon;

  private MotionMagicVoltage m_request =
      new MotionMagicVoltage(PhysicalConstants.ABSOLUTE_ZERO).withEnableFOC(true);
  private VoltageOut m_VoltageOut =
      new VoltageOut(PhysicalConstants.ABSOLUTE_ZERO).withEnableFOC(true);

  // =====Logged Values=====
  StatusSignal<Angle> rightPosition;
  StatusSignal<Voltage> rightAppliedVolts;
  StatusSignal<Current> rightStatorCurrentAmps;
  StatusSignal<Current> rightSupplyCurrentAmps;
  StatusSignal<Temperature> rightTempCelsius;
  StatusSignal<Double> rightSetPosition;
  StatusSignal<AngularVelocity> rightVelocityRPS;

  StatusSignal<Angle> leftPosition;
  StatusSignal<Voltage> leftAppliedVolts;
  StatusSignal<Current> leftStatorCurrentAmps;
  StatusSignal<Current> leftSupplyCurrentAmps;
  StatusSignal<Temperature> leftTempCelsius;
  StatusSignal<Double> leftSetPosition;
  StatusSignal<AngularVelocity> leftVelocityRPS;

  private final BaseStatusSignal[] signals;

  public ElevatorIOReal() {
    rightTalon = new TalonFX(ElevatorConstants.elevatorRightID, Constants.MISC_CANIVORE);
    leftTalon = new TalonFX(ElevatorConstants.elevatorLeftID, Constants.MISC_CANIVORE);

    PhoenixUtil.tryUntilOk(
        5, () -> rightTalon.getConfigurator().apply(ElevatorConstants.elevatorRightTalon));
    leftTalon.setControl(new Follower(ElevatorConstants.elevatorRightID, true));

    rightPosition = rightTalon.getPosition();
    rightAppliedVolts = rightTalon.getMotorVoltage();
    rightStatorCurrentAmps = rightTalon.getTorqueCurrent();
    rightSupplyCurrentAmps = rightTalon.getSupplyCurrent();
    rightTempCelsius = rightTalon.getDeviceTemp();
    rightVelocityRPS = rightTalon.getVelocity();

    leftPosition = leftTalon.getPosition();
    leftAppliedVolts = leftTalon.getMotorVoltage();
    leftStatorCurrentAmps = leftTalon.getTorqueCurrent();
    leftSupplyCurrentAmps = leftTalon.getSupplyCurrent();
    leftTempCelsius = leftTalon.getDeviceTemp();
    leftVelocityRPS = leftTalon.getVelocity();

    signals =
        new BaseStatusSignal[] {
          rightPosition,
          rightAppliedVolts,
          rightStatorCurrentAmps,
          rightSupplyCurrentAmps,
          rightTempCelsius,
          rightVelocityRPS,
          leftPosition,
          leftAppliedVolts,
          leftStatorCurrentAmps,
          leftSupplyCurrentAmps,
          leftTempCelsius,
          leftVelocityRPS
        };

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0,
        rightPosition,
        rightAppliedVolts,
        rightStatorCurrentAmps,
        rightSupplyCurrentAmps,
        rightTempCelsius,
        rightVelocityRPS,
        leftPosition,
        leftAppliedVolts,
        leftStatorCurrentAmps,
        leftSupplyCurrentAmps,
        leftTempCelsius,
        leftVelocityRPS);
    ParentDevice.optimizeBusUtilizationForAll(rightTalon, leftTalon);
  }

  public void updateInputs(ElevatorIOInputs inputs) {
    BaseStatusSignal.refreshAll(signals);

    inputs.rightMotorData =
        new ElevatorIO.RightMotorData(
            BaseStatusSignal.isAllGood(
                rightPosition,
                rightAppliedVolts,
                rightStatorCurrentAmps,
                rightSupplyCurrentAmps,
                rightTempCelsius,
                rightVelocityRPS),
            rightPosition.getValueAsDouble(),
            rightAppliedVolts.getValueAsDouble(),
            rightStatorCurrentAmps.getValueAsDouble(),
            rightSupplyCurrentAmps.getValueAsDouble(),
            rightTempCelsius.getValueAsDouble(),
            rightVelocityRPS.getValueAsDouble());

    inputs.leftMotorData =
        new ElevatorIO.LeftMotorData(
            BaseStatusSignal.isAllGood(
                leftPosition,
                leftAppliedVolts,
                leftStatorCurrentAmps,
                leftSupplyCurrentAmps,
                leftTempCelsius,
                leftVelocityRPS),
            leftPosition.getValueAsDouble(),
            leftAppliedVolts.getValueAsDouble(),
            leftStatorCurrentAmps.getValueAsDouble(),
            leftSupplyCurrentAmps.getValueAsDouble(),
            leftTempCelsius.getValueAsDouble(),
            leftVelocityRPS.getValueAsDouble());
  }

  @Override
  public void setElevatorVoltage(double voltage) {
    rightTalon.setControl(m_VoltageOut.withOutput(voltage));
  }

  @Override
  public void setElevatorTargetPosition(DoubleSupplier positionSupplier) {
    rightTalon.setControl(m_request.withPosition(positionSupplier.getAsDouble()));
  }

  @Override
  public void setElevatorTargetPosition(double position) {
    rightTalon.setControl(m_request.withPosition(position));
  }

  @Override
  public void setElevatorZero() {
    rightTalon.setPosition(0.0);
    leftTalon.setPosition(0.0);
  }

  public void stop() {
    rightTalon.stopMotor();
  }

  @Override
  public boolean checkMotorsStalled() {
    return MotorStallDetection.isMotorStalled(
            rightStatorCurrentAmps.getValueAsDouble(),
            rightVelocityRPS.getValueAsDouble(),
            ElevatorConstants.STALLED_CURRENT,
            ElevatorConstants.STALLED_RPS)
        || MotorStallDetection.isMotorStalled(
            leftStatorCurrentAmps.getValueAsDouble(),
            leftVelocityRPS.getValueAsDouble(),
            ElevatorConstants.STALLED_CURRENT,
            ElevatorConstants.STALLED_RPS);
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
