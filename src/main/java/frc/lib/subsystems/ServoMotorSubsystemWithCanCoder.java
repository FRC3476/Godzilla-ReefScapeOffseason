package frc.lib.subsystems;

import org.littletonrobotics.junction.Logger;

public class ServoMotorSubsystemWithCanCoder<
        T extends MotorInputsAutoLogged,
        U extends MotorIO,
        V extends CanCoderInputsAutoLogged,
        W extends CanCoderIO>
    extends ServoMotorSubsystem<T, U> {
  protected ServoMotorSubsystemWithCanCoderConfig conf;
  protected V cancoderInputs;
  protected W cancoderIO;
  protected boolean hasSetOffset = false;

  public ServoMotorSubsystemWithCanCoder(
      ServoMotorSubsystemWithCanCoderConfig config, T inputs, U io, V cancoderInputs, W cancoder) {
    super(config, inputs, io);
    this.conf = config;
    this.cancoderInputs = cancoderInputs;
    this.cancoderIO = cancoder;
  }

  @Override
  public void periodic() {
    super.periodic();

    cancoderIO.readInputs(cancoderInputs);
    Logger.processInputs(getName() + "/cancoder", cancoderInputs);

    if (!this.conf.isFusedCancoder
        && !this.hasSetOffset
        && !Double.isNaN(cancoderInputs.absolutePositionRotations)) {
      io.setCurrentPosition(cancoderInputs.absolutePositionRotations * conf.cancoderToUnitsRatio);
      this.hasSetOffset = true;
    }
  }

  public void resetOffset() {
    // Don't set boolean above to left main thread still do it as well.
    io.setCurrentPosition(cancoderInputs.absolutePositionRotations * conf.cancoderToUnitsRatio);
  }

  public void resetOffset(int canCoderRotations) {
    io.setCurrentPosition((cancoderInputs.absolutePositionRotations - canCoderRotations) * conf.cancoderToUnitsRatio);
  }

  public void zeroMagnetOffset() {
    cancoderIO.setMagnetOffset(0);
  }

  public void zeroMagnetOffset(double expectedZeroingPositionUnits, ) {
    cancoderIO.setMagnetOffset(0);
    double absoluteRotationsAtZeroingPosition = expectedZeroingPositionUnits * conf.cancoderToUnitsRatio;
    double magnetOffset = 
        absoluteRotationsAtZeroingPosition - cancoderIO.getAbsolutePositionSlow();
    magnetOffset = Util.rangeModulo(
      magnetOffset, 
      conf.canCoderConfig.config.MagnetSensor.AbsoluteSensorDiscontinuityPoint,
      conf.canCoderConfig.config.MagnetSensor.AbsoluteSensorDiscontinuityPoint - 1);

    cancoderIO.setMagnetOffset(magnetOffset);
    cancoderIO.setPosition(cancoderIO.getAbsolutePositionSlow() + Math.round(absoluteRotationsAtZeroingPosition));
  }
}
