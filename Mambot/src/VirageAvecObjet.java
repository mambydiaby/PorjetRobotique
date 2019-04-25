import java.io.File;
import lejos.hardware.*;
import lejos.hardware.ev3.EV3;
import lejos.hardware.motor.Motor;
import lejos.hardware.motor.UnregulatedMotor;
import lejos.hardware.port.MotorPort;
import lejos.utility.Delay;

public class VirageAvecObjet {
	public static void main(String[] args) {
		// Début du programme et attente d'appui sur un bouton par l'utilisateur...
		System.out.println("Press any key to start");
		Sound.beepSequenceUp();   // make sound when ready.
		Delay.msDelay(2000);
		Button.waitForAnyPress();
		
		// Creation de 2 objets Motor pour les 2 roues...
		UnregulatedMotor motorA = new UnregulatedMotor(MotorPort.C);
		UnregulatedMotor motorB = new UnregulatedMotor(MotorPort.D);
		
		// Tourne a gauche de 90 degrés...
		motorA.backward();
        motorB.forward();
        motorA.setPower(55);
        motorB.setPower(55);
        Delay.msDelay(465);
        motorA.stop();
        motorB.stop();
        
        /*Delay.msDelay(250);
        
        // Tourne a droite de 90 degrés...
        motorB.backward();
        motorA.forward();
        motorA.setPower(55);
        motorB.setPower(55);
        Delay.msDelay(465);
        motorA.stop();
        motorB.stop();
        
        Delay.msDelay(250);
        
        // Tourne a droite de 180 degrés...
        motorB.backward();
        motorA.forward();
        motorA.setPower(55);
        motorB.setPower(55);
        Delay.msDelay(930);
        motorA.stop();
        motorB.stop();
        
        Delay.msDelay(250);
        
        // Tourne a gauche de 270 degrés...
        motorA.backward();
        motorB.forward();
        motorA.setPower(55);
        motorB.setPower(55);
        Delay.msDelay(1395);
        motorA.stop();
        motorB.stop();
        
        Delay.msDelay(250);
        
        // Tourne a droite de 90 degrés...
        motorB.backward();
        motorA.forward();
        motorA.setPower(55);
        motorB.setPower(55);
        Delay.msDelay(465);
        motorA.stop();
        motorB.stop();
        */
        Sound.beepSequence();
        Delay.msDelay(250);
        
        // Libère les ressources sur les moteurs... 
        motorA.close(); 
     	motorB.close();
	}
}
