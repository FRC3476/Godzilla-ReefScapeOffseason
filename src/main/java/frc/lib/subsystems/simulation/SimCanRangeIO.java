package frc.lib.subsystems.simulation;

import java.util.function.Supplier;

import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.sim.CANrangeSimState;

import frc.lib.subsystems.canDevice.CanRangeConfig;
import frc.lib.subsystems.canDevice.CanRangeIOHardware;
import frc.lib.subsystems.canDevice.CanRangeInputs;


public class SimCanRangeIO extends CanRangeIOHardware {
    public static class SimCanRangeState{
        public boolean isTripped;
        public double distanceMeters;
    }


    protected CANrangeSimState simState;
    protected Supplier<SimCanRangeState> supplier;

    public SimCanRangeIO(CanRangeConfig config, Supplier<SimCanRangeState> supplier) {
        super(config);
        simState = this.canRange.getSimState();
        this.supplier = supplier;
        // this.simState.setDistance(0);
        this.simState.setSupplyVoltage(0);
    }

    @Override
    public void readInputs(CanRangeInputs inputs) {
        var suppliedState = supplier.get();
        this.simState.setDistance(suppliedState.distanceMeters);
        this.simState.setSupplyVoltage(suppliedState.isTripped ? 1000000.0 : 0.0);

        super.readInputs(inputs);
    }
    
}
