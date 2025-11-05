package frc.robot.subsystems.led;

import frc.robot.Constants.LedConstants.LedStrip;
import frc.robot.util.COColor;
import java.util.function.Supplier;

public interface LedIO {
  class LedInputs {}

  default void readInputs(LedIO.LedInputs inputs) {}

  default void update(final LedIO.LedInputs inputs) {}

  default COColor getCurrentState() {
    return COColor.kOff;
  }
  ;

  default void writePixels(COColor state, LedStrip strip) {}
  ;

  default void writeNumPixels(COColor state, Supplier<Integer> numLeds) {}
  ;

  default void writePixels(COColor[] states) {}
  ;

  default void blink(COColor state, double duration, LedStrip strip) {}
  ;

  default void fire() {}
  ;

  default void rainbow() {}
  ;

  default void colorflowCO() {}
  ;

  default void larson(COColor state) {}
  ;

  default void twinkle(COColor state, boolean off) {}
  ;

  public default void percentageFull(double percent, COColor state) {}
  ;
}
