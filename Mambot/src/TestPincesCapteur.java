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

public class TestPincesCapteur {
	private static EV3TouchSensor ts1 = new EV3TouchSensor(SensorPort.S2);
	public static void main(String[] args) {
		// Début du programme et attente d'appui sur un bouton par l'utilisateur...
		System.out.println("Press any key to start");
		Sound.beepSequenceUp();   // make sound when ready.
		Delay.msDelay(2000);
		Button.waitForAnyPress();
		
		// Creation d'objets Motor pour les roues et les pinces...
		UnregulatedMotor motorC = new UnregulatedMotor(MotorPort.A);
		UnregulatedMotor motorA = new UnregulatedMotor(MotorPort.C);
		UnregulatedMotor motorB = new UnregulatedMotor(MotorPort.D);
		
		// Ouvre les pinces...
		motorC.forward();
		motorC.setPower(100);
		Delay.msDelay(1500);
		motorC.stop();
		
		// Fait avancer le robot tout droit...
		motorA.forward();
        motorB.forward();
        motorA.setPower(50);
        motorB.setPower(50);
        
        int cmpt = 0;
        boolean ok = false;
		while (cmpt < 100 && ok != true) {
			SensorMode touch = ts1.getTouchMode();
			float[] sample = new float[touch.sampleSize()];
			touch.fetchSample(sample, 0);
			if (sample[0] != 0) { // Un objet touche le capteur...				
				// Ferme les pinces...
				motorC.backward();
				motorC.setPower(100);
				Delay.msDelay(1500);
				motorC.stop();
				
				// Stoppe le robot...
				motorA.stop();
				motorB.stop();
				
				// Fait demi-tour puis avance tout droit...
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
		        Delay.msDelay(3000);
		        
		        // Stoppe le robot...
				motorA.stop();
				motorB.stop();
		        
		        // Ouvre les pinces...
				motorC.forward();
				motorC.setPower(100);
				Delay.msDelay(1500);
				motorC.stop();
				
				// Recule...
				motorA.backward();
				motorB.backward();
				Delay.msDelay(2000);
				motorA.stop();
			    motorB.stop();
				
				// Ferme les pinces...
				motorC.backward();
				motorC.setPower(100);
				Delay.msDelay(1500);
				motorC.stop();
				
				ok = true;
			}
			Delay.msDelay(500);
			cmpt++;
		}
		motorA.stop();
	    motorB.stop();
		
		// Libère les ressources sur les moteurs et les pinces... 
        motorA.close(); 
     	motorB.close();
		motorC.close();
		ts1.close();
		
		//Sound.beepSequence();
	}
}
