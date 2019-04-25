package AvecChassis;

import java.awt.Color;
import lejos.hardware.BrickFinder;
import lejos.hardware.Button;
import lejos.hardware.Sound;
import lejos.hardware.ev3.EV3;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.motor.Motor;
import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.EV3TouchSensor;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.hardware.sensor.SensorMode;
import lejos.utility.Delay;

public class TestCouleurs {
	private static EV3TouchSensor ts1 = new EV3TouchSensor(SensorPort.S2);
	private static EV3UltrasonicSensor us1 = new EV3UltrasonicSensor(SensorPort.S1);
	private static EV3ColorSensor cs1 = new EV3ColorSensor(SensorPort.S3);
	private static Pliers pliers = WheeledChassis.modelPliers(Motor.A);
	private static Wheel wheel1 = WheeledChassis.modelWheel(Motor.C, 64).offset(-70);
	private static Wheel wheel2 = WheeledChassis.modelWheel(Motor.D, 64).offset(70);
	private static Chassis chassis = new WheeledChassis(new Wheel[] { wheel1, wheel2 }, new Pliers[] {pliers}, WheeledChassis.TYPE_DIFFERENTIAL);

	public static void main(String[] args) {
		// Début du programme et attente d'appui sur un bouton par l'utilisateur...
		System.out.println("Press any key to start");
		Sound.beepSequenceUp();
		Delay.msDelay(2000);
		Button.waitForAnyPress();
		SensorMode color = cs1.getRGBMode();
		float[] colorSample = new float[3];
		color.fetchSample(colorSample, 0);
		float i1, i2, i3;
		i1 = 255*colorSample[0];
		i2 = 255*colorSample[1];
		i3 = 255*colorSample[2];
		float minR,minG, minB;
		float maxR, maxG, maxB;
		minR = maxR = i1;
		minG = maxG = i2;
		minB = maxB = i3;
		long startTime = System.currentTimeMillis();
		long tps;
		do {
			chassis.travel(10);
			tps = System.currentTimeMillis() - startTime;
			color.fetchSample(colorSample, 0);
			i1 = 255*colorSample[0];
			i2 = 255*colorSample[1];
			i3 = 255*colorSample[2];
			System.out.println("R : " + i1 + "\nG : " + i2 + "\nB : " + i3);
			if (i1 < minR)
				minR = i1;
			else if (i1 > maxR)
				maxR = i1;
			if (i2 < minG)
				minG = i2;
			else if (i2 > maxG)
				maxG = i2;
			if (i3 < minB)
				minB = i3;
			else if (i3 > maxB)
				maxB = i3;
		} while (tps < 10000);
		System.out.println("MaxR : " + maxR + " / MinR : " + minR + "\n"
				+ "MaxG : " + maxG + " / MinG : " + minG + "\n"
				+ "MaxB : " + maxB + " / MinB : " + minB);
		Delay.msDelay(15000);
	}
}
