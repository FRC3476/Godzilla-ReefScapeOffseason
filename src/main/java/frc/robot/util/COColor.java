package frc.robot.util;

import com.ctre.phoenix6.signals.RGBWColor;
import edu.wpi.first.wpilibj.util.Color8Bit;
import java.awt.Color;

public class COColor {
  public static final COColor kRed = new COColor(Color.red);
  public static final COColor kGreen = new COColor(Color.green);
  public static final COColor kBlue = new COColor(Color.blue);
  public static final COColor kCyan = new COColor(Color.cyan);
  public static final COColor kMagenta = new COColor(Color.magenta);
  public static final COColor kYellow = new COColor(Color.yellow);
  public static final COColor kWhite = new COColor(Color.white);
  public static final COColor kBlack = new COColor(Color.black);
  public static final COColor kOff = kBlack;
  public static final COColor kCOTealPure = new COColor(32, 146, 153);
  public static final COColor kCOOrangePure = new COColor(255, 122, 28);
  public static final COColor kCOTealLed = new COColor(0, 100, 35);
  public static final COColor kCOOrangeLed = new COColor(255, 30, 0);

  public static final COColor kOrange = new COColor(255, 80, 0);
  public static final COColor kPurple = new COColor(255, 0, 255);
  public static final COColor kGray = new COColor(128, 128, 128);
  public static final COColor kPink = new COColor(255, 0, 100);
  public static final COColor kLowBattery = kYellow;
  public static final COColor kGoodBattery = kGreen;

  public static final COColor[] kRainbow = {
    kWhite, kWhite, kWhite, kWhite, kWhite, kWhite, kWhite, kWhite, kOff, kRed, kOrange, kYellow,
    kGreen, kCyan, kBlue, kPurple, kOff, kOff
  };

  public static final COColor kCoralMode = kCOOrangeLed;
  public static final COColor kAlgaeMode = kCOTealLed;
  public static final COColor kCoralManual = kWhite;

  public int blue;
  public int green;
  public int red;
  public int alpha;

  public COColor() {
    this(0, 0, 0, 0);
  }

  public COColor(int r, int g, int b, int a) {
    blue = b;
    green = g;
    red = r;
    alpha = a;
  }

  public COColor(int r, int g, int b) {
    this(r, g, b, 255);
  }

  public COColor(Color color) {
    this(color.getRed(), color.getGreen(), color.getBlue(), color.getAlpha());
  }

  public RGBWColor getRGBW() {
    return new RGBWColor(this.red, this.green, this.blue, this.alpha);
  }

  public Color8Bit getColor8Bit() {
    return new Color8Bit(this.red, this.green, this.blue);
  }

  public void copyFrom(COColor other) {
    this.blue = other.blue;
    this.green = other.green;
    this.red = other.red;
  }

  @Override
  public boolean equals(Object other) {
    if (other == null) {
      return false;
    }

    if (other.getClass() != this.getClass()) {
      return false;
    }

    COColor s = (COColor) other;
    return this.blue == s.blue && this.red == s.red && this.green == s.green;
  }

  @Override
  public String toString() {
    String redString = Integer.toHexString(red);
    if (redString.length() == 1) {
      redString = "0" + redString;
    }
    String greenString = Integer.toHexString(green);
    if (greenString.length() == 1) {
      greenString = "0" + greenString;
    }
    String blueString = Integer.toHexString(blue);
    if (blueString.length() == 1) {
      blueString = "0" + blueString;
    }
    return "#" + redString + greenString + blueString;
  }
}
