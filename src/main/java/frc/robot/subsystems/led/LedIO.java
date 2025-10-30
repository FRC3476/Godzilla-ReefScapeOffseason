package frc.robot.subsystems.led;

import frc.robot.Constants.LedConstants.LedStrip;
import java.util.function.Supplier;

public interface LedIO {
  class LedInputs {}

  default void readInputs(LedIO.LedInputs inputs) {}

  default void update(final LedIO.LedInputs inputs) {}

  default LedState getCurrentState() {
    return LedState.kOff;
  }
  ;

  default void writePixels(LedState state, LedStrip strip) {}
  ;

  default void writeNumPixels(LedState state, Supplier<Integer> numLeds) {}
  ;

  default void writePixels(LedState[] states) {}
  ;

  default void blink(LedState state, double duration, LedStrip strip) {}
  ;

  default void fire() {}
  ;

  default void rainbow() {}
  ;

  default void colorflowCO() {}
  ;

  default void larson(LedState state) {}
  ;

  default void twinkle(LedState state, boolean off) {}
  ;

  public default void percentageFull(double percent, LedState state) {}
  ;
}
