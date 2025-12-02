package frc.lib.subsystems.canDevice;

import static edu.wpi.first.units.Units.Meters;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.CANrange;

import edu.wpi.first.units.measure.Distance;
import frc.lib.util.CANStatusLogger;
import frc.lib.util.CTREUtil;

public class CanRangeIOHardware implements CanRangeIO{
    protected final CANrange canrange;
    protected CanRangeConfig config;
    private final BaseStatusSignal[] signals;

    private StatusSignal<Boolean> trippedSignal;
    private StatusSignal<Distance> distanceSignal;

    public CanRangeIOHardware(CanRangeConfig config) {
        this.config = config;

        canrange = new CANrange(config.CANID.getDeviceNumber(), config.CANID.getBusName());

        CTREUtil.applyConfiguration(canrange, this.config.config);
        trippedSignal = canrange.getIsDetected();
        distanceSignal = canrange.getDistance();
        

        signals = new BaseStatusSignal[] {trippedSignal, distanceSignal};

        BaseStatusSignal.setUpdateFrequencyForAll(100.0, signals);

        CANStatusLogger.getInstance()
            .registerCANrange(
                "CANrange_ID" + config.CANID.getDeviceNumber(),
                canrange,
                config.CANID.getDeviceNumber(),
                config.CANID.getBusName());
    }

    @Override
    public void readInputs(CanRangeInputs inputs) {
        BaseStatusSignal.refreshAll(signals);

        inputs.isTripped = trippedSignal.getValue();
        inputs.distanceMeters = distanceSignal.getValue().in(Meters);
        inputs.isConnected = canrange.isConnected();
    }

    @Override
    public void updateFrequency(double hz) {
        BaseStatusSignal.setUpdateFrequencyForAll(hz, signals);
    }

}
