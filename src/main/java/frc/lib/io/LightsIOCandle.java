package frc.lib.io;


import com.ctre.phoenix6.configs.CANdleConfiguration;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.hardware.CANdle;


public class LightsIOCandle extends LightsIO {
	private final CANdle candle;

	public LightsIOCandle(LightsIOCandleConfiguration config) {
		super(config.ledCount);
		candle = new CANdle(config.id, config.bus);
		candle.getConfigurator().apply(config.configuration);
	}

	protected void setLEDs(ControlRequest color) {
		// candle.setLEDs(color.r, color.g, color.b, 0, startIndex, numLeds);
		candle.setControl(color);
	}

	public static class LightsIOCandleConfiguration {
		public int id;
		public String bus;
		public int ledCount;
		public CANdleConfiguration configuration = new CANdleConfiguration();
	}
}
