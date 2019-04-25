import lejos.hardware.Button;
import lejos.hardware.Sound;
import lejos.hardware.motor.*;
import lejos.hardware.port.*;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.robotics.SampleProvider;
import lejos.utility.Delay;

public class TestDistance {
	private static EV3UltrasonicSensor us1 = new EV3UltrasonicSensor(SensorPort.S1);
	
    public static void main(String[] args) {
    	final SampleProvider sp = us1.getDistanceMode();
		float distanceValue = 0;
        final int iteration_threshold = 200;
        for(int i = 0; i <= iteration_threshold; i++) {
        	float [] sample = new float[sp.sampleSize()];
            sp.fetchSample(sample, 0);
            distanceValue = 100*sample[0];
			System.out.println(distanceValue + " cm");
			Delay.msDelay(500);
        }
    }
}