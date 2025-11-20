package frc.lib.io;

import com.ctre.phoenix6.configs.CANdiConfiguration;
import com.ctre.phoenix6.hardware.CANdi;

import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.DigitalInput;

public class BeamBreakIOCANdi extends BeamBreakIO {
	private final CANdi candi; 
    public BeamBreakIOCANdi(int channel, String bus, Time debounce, String name, CANdiConfiguration config) {
        super(debounce, name);
        candi = new CANdi(channel, bus);
        candi.getConfigurator().apply(config); 
    }

    public Boolean getFirstBanner() {
        return candi.getS1Closed().getValue();
    }

    public Boolean getSecondBanner() {
        return candi.getS2Closed().getValue(); 
    }

    @Override
    public boolean get() {
        return getFirstBanner() && getSecondBanner(); 
    }

    
}
