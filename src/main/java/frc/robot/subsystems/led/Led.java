package frc.robot.subsystems.led;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.LedConstants.LedStrip;
import frc.robot.RobotState;
import frc.robot.util.LoggedTunableNumber;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class Led extends SubsystemBase {
  private final LedIO io;
  private final RobotState state;
  private static final LoggedTunableNumber LedsOn = new LoggedTunableNumber("LED/Num On", 0);

  public record PercentageSetpoint(double pct, LedState color) {}

  public Led(final LedIO io, RobotState state) {
    this.io = io;
    this.state = state;
  }

  @Override
  public void periodic() {
    super.periodic();

    RobotState.setLedState(getCurrentState());
    Logger.recordOutput(
        "LED/currentCommand",
        (getCurrentCommand() == null) ? "Default" : getCurrentCommand().getName());
  }

  public LedState getCurrentState() {
    return io.getCurrentState();
  }

  public int getLedsOn() {
    return (int) Math.round(LedsOn.get());
  }

  /* change runOnce to run in case we have to keep setting the LED color periodically? */
  public Command commandSolidColor(LedState state) {
    return run(() -> setSolidColor(state, LedStrip.BOTH))
        .ignoringDisable(true)
        .withName("LED Solid Color");
  }

  public Command commandSolidColor(LedState state, LedStrip strip) {
    return runOnce(() -> setSolidColor(state, strip))
        .ignoringDisable(true)
        .withName("LED Solid Color");
  }

  public Command commandSolidColor(Supplier<LedState> state) {
    return runOnce(() -> setSolidColor(state.get()))
        .ignoringDisable(true)
        .withName("LED Solid Color");
  }

  public Command commandSolidColorNumLeds(LedState state, Supplier<Integer> numLeds) {
    return runOnce(() -> setSolidColorNumLeds(state, numLeds))
        .ignoringDisable(true)
        .withName("LED Solid Color Num Leds");
  }

  public Command commandOff() {
    return commandSolidColor(LedState.kOff).withName("Led Off");
  }

  public Command commandSetTeal() {
    return commandSolidColor(LedState.kCOTealLed).withName("Led Teal");
  }

  public Command commandSetOrange() {
    return commandSolidColor(LedState.kCOOrangeLed).withName("Led Orange");
  }

  public Command commandSolidPattern(LedState[] states) {
    return run(() -> setSolidPattern(states)).ignoringDisable(true).withName("LED Solid Pattern");
  }

  public Command commandPercentageFull(DoubleSupplier percentageFull, LedState state) {
    return run(() -> setPercentageFull(percentageFull.getAsDouble(), state)).ignoringDisable(true);
  }

  public Command commandPercentageFull(Supplier<PercentageSetpoint> percentageSupplier) {
    return run(() ->
            setPercentageFull(percentageSupplier.get().pct, percentageSupplier.get().color))
        .ignoringDisable(true);
  }

  public Command commandBlinkingState(LedState state, double duration) {
    return this.runOnce(() -> blinkingState(state, duration, LedStrip.BOTH)).ignoringDisable(true);
  }

  public Command commandBlinkingState(LedState state, double duration, LedStrip strip) {
    return this.runOnce(() -> blinkingState(state, duration, strip)).ignoringDisable(true);
  }

  public void blinkingState(LedState state, double duration, LedStrip strip) {
    this.io.blink(state, duration, strip);
  }

  public Command commandFire() {
    return this.runOnce(() -> this.io.fire())
        .ignoringDisable(true)
        .ignoringDisable(true)
        .withName("LED Fire");
  }

  public Command commandRainbow() {
    return this.runOnce(() -> this.io.rainbow())
        .ignoringDisable(true)
        .ignoringDisable(true)
        .withName("LED Rainbow");
  }

  public Command commandColorflowCO() {
    return this.runOnce(() -> this.io.colorflowCO())
        .ignoringDisable(true)
        .ignoringDisable(true)
        .withName("LED LarsonCO");
  }

  public Command commandLarson(LedState state) {
    return this.runOnce(() -> this.io.larson(state))
        .ignoringDisable(true)
        .ignoringDisable(true)
        .withName("LED Larson");
  }

  public Command commandTwinkle(LedState state, boolean off) {
    return this.runOnce(() -> this.io.twinkle(state, off))
        .ignoringDisable(true)
        .withName("LED Twinkle");
  }

  private void setSolidColor(LedState state) {
    io.writePixels(state, LedStrip.BOTH);
  }

  private void setSolidColor(LedState state, LedStrip strip) {
    io.writePixels(state, strip);
  }

  private void setSolidColorNumLeds(LedState state, Supplier<Integer> numLeds) {
    System.out.println(numLeds);
    io.writeNumPixels(state, numLeds);
  }

  private void setSolidPattern(LedState[] states) {
    io.writePixels(states);
  }

  // private void setPercentageFull(double percentageFull, LedState state) {
  //   LedState[] pixels = new LedState[Constants.LedConstants.kMaxLEDCount / 2];
  //   for (int i = 0; i < pixels.length; i++) {
  //     if (i < pixels.length * MathUtil.clamp(percentageFull, 0.0, 1.0)) {
  //       pixels[i] = state;
  //     }
  //   }
  // }

  private void setPercentageFull(double percent, LedState state) {
    this.io.percentageFull(percent, state);
  }

  @SuppressWarnings("unused")
  private LedState[] mirror(LedState[] pixels) {
    LedState[] fullPixels = new LedState[Constants.LedConstants.kMaxLEDCount];

    for (int i = Constants.LedConstants.kCandleLEDCount;
        i
            < Constants.LedConstants.kCandleLEDCount
                + (Constants.LedConstants.kNonCandleLEDCount / 2);
        i++) {
      fullPixels[fullPixels.length - i - 2] = pixels[i];
      fullPixels[i] = pixels[i];
    }

    return fullPixels;
  }
}
