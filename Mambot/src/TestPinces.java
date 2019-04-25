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

public class TestPinces {
	public static void main(String[] args) {
		// Début du programme et attente d'appui sur un bouton par l'utilisateur...
		System.out.println("Press any key to start");
		Sound.beepSequenceUp();   // make sound when ready.
		Delay.msDelay(2000);
		Button.waitForAnyPress();
		
		// Creation d'un objet Motor pour les pinces...
		UnregulatedMotor motorC = new UnregulatedMotor(MotorPort.A);
		
		motorC.forward();
		motorC.setPower(100);
		Delay.msDelay(1500);
		motorC.backward();
		Delay.msDelay(1500);
		motorC.stop();
		motorC.close();
	}
}
