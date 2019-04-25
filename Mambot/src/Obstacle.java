import java.io.File;
import lejos.hardware.*;
import lejos.hardware.ev3.EV3;
import lejos.hardware.motor.Motor;
import lejos.hardware.motor.UnregulatedMotor;
import lejos.hardware.port.MotorPort;
import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.robotics.SampleProvider;
import lejos.utility.Delay;

public class Obstacle {
	// Creation d'un objet Sensor pour le "radar"...
	private static EV3UltrasonicSensor us1 = new EV3UltrasonicSensor(SensorPort.S1);
	public static void main(String[] args) {
		// Début du programme et attente d'appui sur un bouton par l'utilisateur...
		System.out.println("Press any key to start");
		Sound.beepSequenceUp();   // make sound when ready.
		Delay.msDelay(2000);
		Button.waitForAnyPress();
		
		// Creation de 2 objets Motor pour les 2 roues...
		UnregulatedMotor motorA = new UnregulatedMotor(MotorPort.C);
		UnregulatedMotor motorB = new UnregulatedMotor(MotorPort.D);
		
		motorA.forward();
        motorB.forward();
        motorA.setPower(50);
        motorB.setPower(50);
		int cmpt = 0;
		while (cmpt < 50) {
			SampleProvider sp = us1.getDistanceMode();
			float [] sample = new float[sp.sampleSize()];
			sp.fetchSample(sample, 0);
			if (100*sample[0] < 30) {
				// Tourne a droite de 180 degrés...
		        motorB.backward();
		        motorA.forward();
		        motorA.setPower(55);
		        motorB.setPower(55);
		        Delay.msDelay(930);
		        motorA.stop();
		        motorB.stop();
		        Delay.msDelay(1);
		        motorA.forward();
		        motorB.forward();
			}
			Delay.msDelay(500);
			cmpt++;
		}
		motorA.stop();
	    motorB.stop();
	    
	    // Libère les ressources sur les moteurs et le radar... 
        motorA.close(); 
     	motorB.close();
     	us1.close();
     	
     	Sound.beepSequence();
	}
}
