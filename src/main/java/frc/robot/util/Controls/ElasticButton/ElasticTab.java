package frc.robot.util.Controls.ElasticButton;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import java.util.HashMap;
import java.util.Map;

public class ElasticTab {
  private Map<String, ElasticButton> buttons = new HashMap<>();
  private String key;
  private NetworkTable table;

  public ElasticTab(String key) {
    this.key = key;
    table = NetworkTableInstance.getDefault().getTable("Elastic/" + key);
  }

  public ElasticButton addButton(String key) {
    ElasticButton button = new ElasticButton(this, key, false);
    buttons.put(key, button);
    return button;
  }

  public ElasticButton addButton(String key, boolean defaultValue) {
    ElasticButton button = new ElasticButton(this, key, defaultValue);
    buttons.put(key, button);
    return button;
  }

  public NetworkTable getTable() {
    return table;
  }

  public ElasticButton getButton(String key) {
    return buttons.get(key);
  }
}
