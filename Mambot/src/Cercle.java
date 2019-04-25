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

public class Cercle {
	public static void main(String[] args) {
		// Début du programme et attente d'appui sur un bouton par l'utilisateur...
		/*System.out.println("Press any key to start");
		Sound.beepSequenceUp();   // make sound when ready.
		Delay.msDelay(2000);
		Button.waitForAnyPress();*/
		
		// Creation de 2 objets Motor pour les 2 roues...
		UnregulatedMotor motorA = new UnregulatedMotor(MotorPort.C);
		UnregulatedMotor motorB = new UnregulatedMotor(MotorPort.D);

		motorA.forward();
		motorB.forward();
		
		// Tourne à droite...
		motorA.setPower(70);
		motorB.setPower(30);
		
		Delay.msDelay(3000);
		
		motorA.stop();
	    motorB.stop();
	    
	    // Libère les ressources sur les moteurs et le radar... 
        motorA.close(); 
     	motorB.close();
     	
     	Sound.beepSequence();
	}
}
