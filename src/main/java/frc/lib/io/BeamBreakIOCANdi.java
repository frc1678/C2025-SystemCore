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
    private final int channel; 

	public static BeamBreakIOCANdi makeInverted(int channel, Time debounce, String name, CANdi candi) {
		return new BeamBreakIOCANdi(channel, debounce, name, candi) {
			@Override
			public boolean get() {
				return !super.get();
			}
		};
	}

	public BeamBreakIOCANdi(int channel, Time debounce, String name, CANdi candi) {
		super(debounce, name);

        assert channel == 1 || channel == 2; //Check to make sure that the channel # is valid

		this.mCANdi = candi; 

        this.channel = channel; 
        
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
        return channel == 1 ? getSignalInput1() : getSignalInput2();
    } 
}
