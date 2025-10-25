package frc.robot.util.Controls.ElasticButton;

import java.util.HashSet;
import java.util.Map;
import java.util.Set;

import edu.wpi.first.math.Pair;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class ElasticTab {
    private Map<String,ElasticButton> buttons;
    private String key;
    private NetworkTable table;

    public ElasticTab(String key) {
        this.key = key;
        table = NetworkTableInstance.getDefault().getTable("Elastic/"+key);
    }

    // public ElasticTab(String key, Set<Pair<String,Boolean>> buttonKeyDefaultPair) {
    //     for (Pair<String,Boolean> p:buttonKeyDefaultPair) {
    //         buttons.put(p.getFirst(), new ElasticButton(this, p.getFirst(), p.getSecond()));
    //     }
    // }

    // public static ElasticTab Default(String key, Set<String> buttonKeys) {
    //     Set<Pair<String,Boolean>> keyDefaultPair = new HashSet<>();
    //     for (String k:buttonKeys) {
    //         keyDefaultPair.add(new Pair<String,Boolean>(k, false));
    //     }
    //     return new ElasticTab(key, keyDefaultPair);
    // }

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
