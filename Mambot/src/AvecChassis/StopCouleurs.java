package AvecChassis;

import java.awt.Color;
import lejos.hardware.BrickFinder;
import lejos.hardware.Button;
import lejos.hardware.ev3.EV3;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.motor.Motor;
import lejos.hardware.motor.UnregulatedMotor;
import lejos.hardware.port.MotorPort;
import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.EV3TouchSensor;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.hardware.sensor.SensorMode;
import lejos.utility.Delay;

public class StopCouleurs {
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
		//Sound.beepSequenceUp();   // make sound when ready.
		Delay.msDelay(2000);
		Button.waitForAnyPress();

		//UnregulatedMotor motorA = new UnregulatedMotor(MotorPort.C);
		//UnregulatedMotor motorB = new UnregulatedMotor(MotorPort.D);

		SensorMode color = cs1.getRGBMode();
		float[] colorSample = new float[3];
		color.fetchSample(colorSample, 0);
		float i1, i2, i3;
		i1 = 255*colorSample[0];
		i2 = 255*colorSample[1];
		i3 = 255*colorSample[2];
		long startTime = System.currentTimeMillis();
		long tps;
		boolean ok = true;
		//chassis.travel(40);
		//chassis.setAngularSpeed(40);
		do {
			chassis.travel(100);
			chassis.setAngularSpeed(100);
			tps = System.currentTimeMillis() - startTime;
			color.fetchSample(colorSample, 0);
			i1 = 255*colorSample[0];
			i2 = 255*colorSample[1];
			i3 = 255*colorSample[2];
			if (i1 < 50 && i1 > 33
					&& i2 < 58 && i2 > 35
					&& i3 < 39 && i3 > 22) {
				System.out.println("BLANC");
				//Button.LEDPattern(1);
			}

			else if (i1 < 14 && i1 > 4.5
					&& i2 < 35 && i2 > 16
					&& i3 < 30 && i3 > 12) {
				System.out.println("BLEU");
				//Button.LEDPattern(1);
			}

			else if (i1 < 5 && i1 > 2.5
					&& i2 < 7.5 && i2 > 4
					&& i3 < 4.5 && i3 > 2) {
				System.out.println("NOIR");
				//Button.LEDPattern(0);
			}

			else if (i1 < 59 && i1 > 32
					&& i2 < 54 && i2 > 32
					&& i3 < 10 && i3 > 7) {
				System.out.println("JAUNE");
				//Button.LEDPattern(3);
			}

			else if (i1 < 45 && i1 > 23
					&& i2 < 11 && i2 > 6
					&& i3 < 6 && i3 > 2) {
				System.out.println("ROUGE");
				//Button.LEDPattern(2);
			}

			else if (i1 < 20.5 && i1 > 9
					&& i2 < 45 && i2 > 22
					&& i3 < 12 && i3 > 5) {
				System.out.println("VERT");
				//Button.LEDPattern(1);
				chassis.setAngularSpeed(0);
				chassis.stop();
				chassis.setAngularSpeed(100);
				chassis.rotate(90);
				ok = false;
			}

			else
				System.out.println("AUTRE");

			Delay.msDelay(10);
		} while ((tps < 30000) && (ok == true));
		chassis.waitComplete();
		chassis.stop();
	}
}
