package frc.robot.util.Controls;

import java.util.ArrayList;
import java.util.List;

public class StreamDeckButton {

  private final int index;
  private final String key;
  private String activeBackground = "#000000";
  private String inactiveBackground = "#000000";
  private String activeForeground = "#FFFFFF";
  private String inactiveForeground = "#FFFFFF";
  private String activeText = "";
  private String inactiveText = "";
  private boolean activeSet = false;
  private boolean inactiveSet = false;

  public StreamDeckButton(int row, int col, String key) {
    index = row * 8 + col % 8;
    this.key = key;
  }

  private void setInactiveIfUnconfigured() {
    if (!this.inactiveSet) {
      this.inactiveBackground = this.activeBackground;
      this.inactiveForeground = this.activeForeground;
      this.inactiveText = this.activeText;
    }
  }

  private void setActiveIfUnconfigured() {
    if (!this.activeSet) {
      this.activeBackground = this.inactiveBackground;
      this.activeForeground = this.inactiveForeground;
      this.activeText = this.inactiveText;
    }
  }

  public StreamDeckButton withActiveConfig(
      String active_background, String active_foreground, String active_text) {
    this.activeBackground = active_background;
    this.activeForeground = active_foreground;
    this.activeText = active_text;
    this.activeSet = true;
    setInactiveIfUnconfigured();
    return this;
  }

  public StreamDeckButton withActiveConfig(StreamDeckButtonConfig streamDeckText) {
    this.activeBackground = streamDeckText.getBackground();
    this.activeForeground = streamDeckText.getForeground();
    this.activeText = streamDeckText.getText();
    this.activeSet = true;
    setInactiveIfUnconfigured();
    return this;
  }

  public StreamDeckButton withInactiveConfig(
      String inactive_background, String inactive_foreground, String inactive_text) {
    this.inactiveBackground = inactive_background;
    this.inactiveForeground = inactive_foreground;
    this.inactiveText = inactive_text;
    this.inactiveSet = true;
    setActiveIfUnconfigured();
    return this;
  }

  public StreamDeckButton withInactiveConfig(StreamDeckButtonConfig streamDeckText) {
    this.inactiveBackground = streamDeckText.getBackground();
    this.inactiveForeground = streamDeckText.getForeground();
    this.inactiveText = streamDeckText.getText();
    this.inactiveSet = true;
    setActiveIfUnconfigured();
    return this;
  }

  public StreamDeckButton withActiveBackground(String background) {
    this.activeBackground = background;
    return this;
  }

  public StreamDeckButton withInactiveBackground(String background) {
    this.inactiveBackground = background;
    return this;
  }

  public StreamDeckButton withActiveForeground(String foreground) {
    this.activeForeground = foreground;
    return this;
  }

  public StreamDeckButton withInactiveForeground(String foreground) {
    this.inactiveForeground = foreground;
    return this;
  }

  public StreamDeckButton withActiveText(String text) {
    this.activeText = text;
    return this;
  }

  public StreamDeckButton withInactiveText(String text) {
    this.inactiveText = text;
    return this;
  }

  public StreamDeckButton withText(String text) {
    this.activeText = text;
    this.inactiveText = text;
    return this;
  }

  public int getIndex() {
    return index;
  }
  
  public static List<String> getNetworkTableKeys() {
    List<String> ntKeys = new ArrayList<>();
    ntKeys.add("Key");
    ntKeys.add("ActiveBackground");
    ntKeys.add("InactiveBackground");
    ntKeys.add("ActiveForeground");
    ntKeys.add("InactiveForeground");
    ntKeys.add("ActiveText");
    ntKeys.add("InactiveText");
    return ntKeys;
  }

  public List<String> getDataToPublish() {
    List<String> dataToPublish = new ArrayList<>();
    dataToPublish.add("StreamDeckButton/" + key);
    dataToPublish.add(activeBackground);
    dataToPublish.add(inactiveBackground);
    dataToPublish.add(activeForeground);
    dataToPublish.add(inactiveForeground);
    dataToPublish.add(activeText);
    dataToPublish.add(inactiveText);
    return dataToPublish;
  }
}
