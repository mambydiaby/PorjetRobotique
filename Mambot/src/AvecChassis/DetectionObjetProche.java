package AvecChassis;

import java.io.File;
import lejos.hardware.*;
import lejos.hardware.ev3.EV3;
import lejos.hardware.motor.Motor;
import lejos.hardware.motor.UnregulatedMotor;
import lejos.hardware.port.MotorPort;
import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.EV3TouchSensor;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.hardware.sensor.SensorMode;
import lejos.robotics.SampleProvider;
import lejos.utility.Delay;

public class DetectionObjetProche {
	private static EV3TouchSensor ts1 = new EV3TouchSensor(SensorPort.S2);
	private static EV3UltrasonicSensor us1 = new EV3UltrasonicSensor(SensorPort.S1);
	private static Pliers pliers = WheeledChassis.modelPliers(Motor.A);
	private static Wheel wheel1 = WheeledChassis.modelWheel(Motor.C, 64).offset(-70);
	private static Wheel wheel2 = WheeledChassis.modelWheel(Motor.D, 64).offset(70);
	private static Chassis chassis = new WheeledChassis(new Wheel[] { wheel1, wheel2 }, new Pliers[] {pliers}, WheeledChassis.TYPE_DIFFERENTIAL);
	
	public static void attrapeObjet() {
		
		// Fait avancer le robot tout droit...
		chassis.travel(1000);

		int cmpt = 0;
		boolean ok = false, ok2 = false;

		// Boucle pour repérer l'objet grace au "radar"
		final SampleProvider sp = us1.getDistanceMode();
		float [] sample2 = new float[sp.sampleSize()];
		while (cmpt < 100 && ok2 != true) {
			sp.fetchSample(sample2, 0);
			if (100*sample2[0] < 40) {
				// Ouvre les pinces...
				chassis.openPliers();
				ok2 = true;
			}
			Delay.msDelay(500);
			cmpt++;
		}        

		while (cmpt < 100 && ok != true) {
			SensorMode touch = ts1.getTouchMode();
			float[] sample = new float[touch.sampleSize()];
			touch.fetchSample(sample, 0);
			if (sample[0] != 0) { // Un objet touche le capteur...				
				// Ferme les pinces...
				Delay.msDelay(500);
				chassis.stop();
				chassis.closePliers();

				// Stop le robot...
				

				// Fait demi-tour puis avance tout droit...
				chassis.setAngularSpeed(300);
				chassis.rotate(180);
				chassis.waitComplete();
				chassis.travel(500);
				chassis.waitComplete();

				// Stoppe le robot...
				chassis.stop();

				// Ouvre les pinces...
				chassis.openPliers();

				// Recule...
				chassis.travel(-300);
				chassis.waitComplete();
				chassis.stop();

				// Ferme les pinces...
				chassis.closePliers();

				ok = true;
			}
			Delay.msDelay(500);
			cmpt++;
		}
		chassis.stop();
	}

	public static void detecteObjet() {		
		final SampleProvider sp = us1.getDistanceMode();
		float [] sample = new float[sp.sampleSize()];

		float min = 10000;
		chassis.setAngularSpeed(200);
		chassis.rotate(360);

		while(chassis.isMoving()) {
			sp.fetchSample(sample, 0);
			if (100*sample[0] < min)
				min = 100*sample[0];
			Delay.msDelay(50);
		}
		chassis.waitComplete();
		chassis.setAngularSpeed(10);
		chassis.rotate(360);
		sp.fetchSample(sample, 0);
		while(100*sample[0] > min) {
			sp.fetchSample(sample, 0);
			Delay.msDelay(50);
		}
		chassis.stop();
		//chassis.travel(500);
		//chassis.waitComplete();
	}

	public static void main(String[] args) {
		// Début du programme et attente d'appui sur un bouton par l'utilisateur...
		System.out.println("Press any key to start");
		Sound.beepSequenceUp();   // make sound when ready.
		Delay.msDelay(2000);
		Button.waitForAnyPress();

		detecteObjet();
		attrapeObjet();
		//chassis.openPliers();
		//chassis.closePliers();
	}
}
