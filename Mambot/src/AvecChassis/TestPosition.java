package AvecChassis;

import lejos.hardware.Button;
import lejos.hardware.Sound;
import lejos.hardware.motor.Motor;
import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.EV3TouchSensor;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.robotics.geometry.Point;
import lejos.robotics.localization.OdometryPoseProvider;
import lejos.robotics.localization.PoseProvider;
import lejos.robotics.navigation.Pose;
import lejos.utility.Delay;

public class TestPosition {
	private static EV3TouchSensor ts1 = new EV3TouchSensor(SensorPort.S2);
	private static EV3UltrasonicSensor us1 = new EV3UltrasonicSensor(SensorPort.S1);
	private static Pliers pliers = WheeledChassis.modelPliers(Motor.A);
	private static Wheel wheel1 = WheeledChassis.modelWheel(Motor.C, 64).offset(-70);
	private static Wheel wheel2 = WheeledChassis.modelWheel(Motor.D, 64).offset(70);
	private static Chassis chassis = new WheeledChassis(new Wheel[] { wheel1, wheel2 }, new Pliers[] {pliers}, WheeledChassis.TYPE_DIFFERENTIAL);
	private static PoseProvider pp = chassis.getPoseProvider();
	private static Pose pose = pp.getPose();
	private static Point location = pose.getLocation();
	
	
	public static void maj() {
		pp = chassis.getPoseProvider();
		pose = pp.getPose();
		location = pose.getLocation();
		float posX1 = location.x;
		float posY1 = location.y;
		chassis.travel(100);
		chassis.waitComplete();
		pp = chassis.getPoseProvider();
		pose = pp.getPose();
		location = pose.getLocation();
		float posX2 = location.x;
		float posY2 = location.y;
		float abc = (float) (90-( (float) Math.atan((posX2 - posX1) / (posY2 - posY1))*57.2958));
		chassis.rotate(-1*abc);
		chassis.waitComplete();
	}
	
	public static void travelTo(float x, float y) {
		pp = chassis.getPoseProvider();
		pose = pp.getPose();
		location = pose.getLocation();
		float posX1 = location.x;
		float posY1 = location.y;
		float angle = (float) (Math.atan((y - posY1) / (x - posX1))*57.2958);
		chassis.rotate(angle);
		chassis.waitComplete();
		float dist = (float) Math.sqrt((y - posY1)*(y - posY1) + (x - posX1)*(x - posX1));
		chassis.travel(dist);
		chassis.waitComplete();
		chassis.rotate(-1*angle);
		chassis.waitComplete();
	}
	
	public static void main(String[] args) {
		// Début du programme et attente d'appui sur un bouton par l'utilisateur...
		//System.out.println("Press any key to start");
		//Sound.beepSequenceUp();   // make sound when ready.
		//Delay.msDelay(2000);
		//Button.waitForAnyPress();

		/*long startTime = System.currentTimeMillis();
		long tps;
		chassis.travel(1500);
		chassis.waitComplete();
		chassis.rotate(90);
		chassis.waitComplete();
		chassis.travel(500);
		do {
			tps = System.currentTimeMillis() - startTime;
			
			pp = chassis.getPoseProvider();
			pose = pp.getPose();
			location = pose.getLocation();
			float posX = location.x;
			float posY = location.y;
			System.out.println("X : " + posX + " --- Y : " + posY);
			
			
			Delay.msDelay(100);
		} while (tps < 25000);*/
		chassis.travel(650);
		chassis.waitComplete();
		chassis.rotate(73);
		chassis.waitComplete();
		chassis.travel(50);
		chassis.waitComplete();
		maj();
		chassis.stop();
		travelTo(1500, 500);
	}
}
