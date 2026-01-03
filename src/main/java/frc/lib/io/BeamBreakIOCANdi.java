package frc.lib.io;

import com.ctre.phoenix6.configs.CANdiConfiguration;
import com.ctre.phoenix6.hardware.CANdi;
import com.ctre.phoenix6.signals.S1CloseStateValue;
import com.ctre.phoenix6.signals.S1FloatStateValue;
import com.ctre.phoenix6.signals.S2CloseStateValue;
import com.ctre.phoenix6.signals.S2FloatStateValue;

import edu.wpi.first.units.measure.Time;

public class BeamBreakIOCANdi extends BeamBreakIO {
	private final CANdi mCANdi;
    private final boolean isChannelOne; 

	public static BeamBreakIOCANdi makeInverted(boolean isChannelOne, Time debounce, String name, CANdi candi) {
		return new BeamBreakIOCANdi(isChannelOne, debounce, name, candi) {
			@Override
			public boolean get() {
				return !super.get();
			}
		};
	}

	public BeamBreakIOCANdi(boolean isChannelOne, Time debounce, String name, CANdi candi) {
		super(debounce, name);
		this.mCANdi = candi; 

        this.isChannelOne = isChannelOne; 
        
        CANdiConfiguration candiConfiguration = new CANdiConfiguration(); 

        candiConfiguration.DigitalInputs.S1CloseState = S1CloseStateValue.CloseWhenNotHigh;
        candiConfiguration.DigitalInputs.S1FloatState = S1FloatStateValue.PullLow;
        candiConfiguration.DigitalInputs.S2CloseState = S2CloseStateValue.CloseWhenNotHigh;
        candiConfiguration.DigitalInputs.S2FloatState = S2FloatStateValue.PullLow;

        mCANdi.getConfigurator().apply(candiConfiguration); 
	}

    public boolean getSignalInput1() {
        return mCANdi.getS1Closed().getValue(); 
    }

	public boolean getSignalInput2() {
        return mCANdi.getS2Closed().getValue(); 
    };

    @Override
    public boolean get() {
        return isChannelOne ? getSignalInput1() : getSignalInput2();
    } 
}
