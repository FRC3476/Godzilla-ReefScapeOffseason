package frc.robot.subsystems.led;

import frc.robot.Constants.LedConstants.LedStrip;
import frc.robot.util.Color;
import java.util.function.Supplier;

public interface LedIO {
  class LedInputs {}

  default void readInputs(LedIO.LedInputs inputs) {}

  default void update(final LedIO.LedInputs inputs) {}

  default Color getCurrentState() {
    return Color.kOff;
  }
  ;

  default void writePixels(Color state, LedStrip strip) {}
  ;

  default void writeNumPixels(Color state, Supplier<Integer> numLeds) {}
  ;

  default void writePixels(Color[] states) {}
  ;

  default void blink(Color state, double duration, LedStrip strip) {}
  ;

  default void fire() {}
  ;

  default void rainbow() {}
  ;

  default void colorflowCO() {}
  ;

  default void larson(Color state) {}
  ;

  default void twinkle(Color state, boolean off) {}
  ;

  public default void percentageFull(double percent, Color state) {}
  ;
}
