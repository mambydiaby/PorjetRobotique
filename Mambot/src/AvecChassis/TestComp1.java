package AvecChassis;
import java.io.File;
import lejos.hardware.*;
import lejos.hardware.ev3.EV3;
import lejos.hardware.motor.Motor;
import lejos.hardware.motor.UnregulatedMotor;
import lejos.hardware.port.MotorPort;
import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.EV3TouchSensor;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.hardware.sensor.SensorMode;
import lejos.robotics.SampleProvider;
import lejos.robotics.geometry.Point;
import lejos.robotics.localization.PoseProvider;
import lejos.robotics.navigation.Pose;
import lejos.utility.Delay;


public class TestComp1 {
	private static EV3TouchSensor ts1 = new EV3TouchSensor(SensorPort.S2);
	private static EV3UltrasonicSensor us1 = new EV3UltrasonicSensor(SensorPort.S1);
	private static Pliers pliers = WheeledChassis.modelPliers(Motor.A);
	private static Wheel wheel1 = WheeledChassis.modelWheel(Motor.C, 64).offset(-70);
	private static Wheel wheel2 = WheeledChassis.modelWheel(Motor.D, 64).offset(70);
	private static Chassis chassis = new WheeledChassis(new Wheel[] { wheel1, wheel2 }, new Pliers[] {pliers}, WheeledChassis.TYPE_DIFFERENTIAL);
	private static EV3ColorSensor cs1 = new EV3ColorSensor(SensorPort.S3);
	private static PoseProvider pp = chassis.getPoseProvider();
	private static Pose pose = pp.getPose();
	private static Point location = pose.getLocation();


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
			if (100*sample2[0] < 40)
				ok2 = true;
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
		chassis.stop();
		chassis.setAngularSpeed(200);
		chassis.rotate(360);
		while(chassis.isMoving()) {
			sp.fetchSample(sample, 0);
			if (100*sample[0] < min)
				min = 100*sample[0];
			Delay.msDelay(50);
		}
		chassis.waitComplete();
		chassis.setAngularSpeed(20);
		chassis.rotate(360);
		sp.fetchSample(sample, 0);
		while(100*sample[0] > min) {
			sp.fetchSample(sample, 0);
			Delay.msDelay(50);
		}
		chassis.stop();
	}

	public static int afficheCouleur(){
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
		//System.out.println("R : " + colorSample[0] + "\nG : " + colorSample[1] + "\nB : " + colorSample[2]);
		long startTime = System.currentTimeMillis();
		long tps;
		//chassis.travel(2000);
		//chassis.setLinearSpeed(0.1);
		//chassis.setLinearAcceleration(0.1);
		tps = System.currentTimeMillis() - startTime;
		color.fetchSample(colorSample, 0);
		i1 = 255*colorSample[0];
		i2 = 255*colorSample[1];
		i3 = 255*colorSample[2];
		if (i1 < 50 && i1 > 33
				&& i2 < 58 && i2 > 35
				&& i3 < 39 && i3 > 22) {
			System.out.println("BLANC");
			Button.LEDPattern(1);
			return 1;
		}

		else if (i1 < 14 && i1 > 4
				&& i2 < 35 && i2 > 14
				&& i3 < 30 && i3 > 12) {
			System.out.println("BLEU");
			Button.LEDPattern(1);
			return 2;
		}

		else if (i1 < 12 && i1 > 2.5
				&& i2 < 15 && i2 > 4
				&& i3 < 7 && i3 > 2) {
			System.out.println("NOIR");
			Button.LEDPattern(0);
			return 3;
		}

		else if (i1 < 59 && i1 > 32
				&& i2 < 54 && i2 > 32
				&& i3 < 10 && i3 > 7) {
			System.out.println("JAUNE");
			Button.LEDPattern(3);
			return 4;
		}

		else if (i1 < 35 && i1 > 22
				&& i2 < 9 && i2 > 5.5
				&& i3 < 5 && i3 > 3) {
			System.out.println("ROUGE");
			Button.LEDPattern(2);
			return 5;
		}

		else if (i1 < 12.5 && i1 > 7.5
				&& i2 < 30 && i2 > 18
				&& i3 < 10 && i3 > 4) {
			System.out.println("VERT");
			Button.LEDPattern(1);
			return 6;
		}

		else {
			System.out.println("AUTRE");
			return 0;
		}
	} 

	public static void followline(int n) { // suit la ligne de la couleur n 
		int i=1;
		int p=0;
		int q=0;
		int r=1;
		while(q<3) {
			while(p<10) {
				if(afficheCouleur()!=n) {
					chassis.setAngularSpeed(110);
					chassis.rotate(i*r);
					chassis.waitComplete();
					chassis.stop();
					i=i*-1;
					p=0;
					r+=1;
				}
				p+=1;
			}
			chassis.travel(30);
			chassis.waitComplete();
			chassis.stop();
			if(afficheCouleur()==n)
				q+=1;
			else{
				p=0;
				r=10;
				q=0;
			}
		}
	}

	public static void rotation(int vitesse, int angle) {
		chassis.setAngularSpeed(vitesse);
		chassis.rotate(angle);
		chassis.waitComplete();
		chassis.stop();
	}

	public static void avance(int dist) {
		chassis.setLinearSpeed(chassis.getMaxLinearSpeed());
		chassis.travel(dist);
		chassis.waitComplete();
		chassis.stop();
	}

	public static void avanceLigne(int couleur) {
		while(afficheCouleur()!=couleur) { 
			chassis.setLinearSpeed(chassis.getMaxLinearSpeed());
			chassis.travel(100);
		}
	}

	public static void AuntilDetec() {
		chassis.setLinearSpeed(6000);
		while(true) {

			SensorMode touch = ts1.getTouchMode();
			float[] sample = new float[touch.sampleSize()];
			touch.fetchSample(sample, 0);
			chassis.travel(200);

			if (sample[0] != 0) { // Un objet touche le capteur...				
				// Ferme les pinces...
				Delay.msDelay(500);
				chassis.stop();
				chassis.closePliers();
				break;
			}
		}
		chassis.waitComplete();
		chassis.stop();
	}

	public static int ADouL(int couleur, int cas1, int cas2) { // avance jusqu'a detection ou detection couleur
		while(true) {

			SensorMode touch = ts1.getTouchMode();
			float[] sample = new float[touch.sampleSize()];
			touch.fetchSample(sample, 0);

			chassis.travel(100);
			if (sample[0] != 0) { // Un objet touche le capteur...				
				// Ferme les pinces...
				Delay.msDelay(500);
				chassis.stop();
				chassis.closePliers(); 
				return cas1;
			}
			if(afficheCouleur()==couleur) {// si pas detection mais ligne noir
				chassis.closePliers();
				return cas2;
			}
		}
	}

	public static void depose() {
		chassis.setLinearSpeed(chassis.getMaxLinearSpeed());
		while(afficheCouleur()!=1) { // tant pas couleur blanche avance
			chassis.travel(200);
		}
		chassis.openPliers();
		avance(-150); //recule
	}

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

	public static void main(String[] args) {//travail sur celui là

		System.out.println("Press any key to start");
		Delay.msDelay(2000);
		Button.waitForAnyPress();
		chassis.openPliers();
		AuntilDetec();


		chassis.setLinearSpeed(chassis.getMaxLinearSpeed());
		rotation(100,-55); // rotation vers la gauche
		avance(500);
		rotation(100,52); // rotation vers la droite
		depose();
		rotation(100, 180);
		avanceLigne(6);
		avance(100);
		rotation(200,-90);
		///followline(6); //s'aligne sur ligne blue


		AuntilDetec();
		rotation(100, -90);
		chassis.stop();
		depose();
		rotation(100, 180);

		avanceLigne(6);
		avance(100);
		rotation(20,-90);
		///followline(6); 

		AuntilDetec();
		rotation(100, -90);
		chassis.stop();
		avanceLigne(1);
		chassis.openPliers();
		avance(-250);
		rotation(100,90);
		avance(-250);

		avanceLigne(4);
		avance(100);
		rotation(100,90);

		followline(4); // suivre ligne rouge
		int A=ADouL(3,1,2);


		rotation(150, 180);
		avanceLigne(A);
		chassis.closePliers();
		chassis.openPliers();
		avance(-250);
		rotation(100,-90);
		avanceLigne(3);
		avance(100);
		rotation(100,-90);
		followline(3);
		int B=ADouL(2, 1, 2);


		rotation(150, 180);
		avanceLigne(B);
		chassis.openPliers();
		avance(-250);
		rotation(100,-90);


		avanceLigne(5);
		avance(100);
		rotation(120,-90);
		followline(5);
		int C=0;
		while(true) {

			SensorMode touch = ts1.getTouchMode();
			float[] sample = new float[touch.sampleSize()];
			touch.fetchSample(sample, 0);
			chassis.setLinearSpeed(chassis.getMaxLinearSpeed());
			chassis.travel(100);
			if (sample[0] != 0) { // Un objet touche le capteur...				
				// Ferme les pinces...
				Delay.msDelay(500);
				chassis.stop();
				chassis.closePliers(); 
				C=0;
				break;
			}
			if(afficheCouleur()==2) {
				chassis.closePliers();
				C=1;
				break;
			}
		}
		if(C==0) {
			rotation(150, 180);
			while(afficheCouleur()!=1) {// on le dépose 
				chassis.setLinearSpeed(chassis.getMaxLinearSpeed());
				chassis.travel(100);
			}
			chassis.openPliers();
			avance(-250);
			rotation(150, 180);
			chassis.closePliers();
			avanceLigne(2);
		}
		chassis.openPliers();
		avance(100);
		rotation(100,-90);
		followline(2);
		AuntilDetec();
		rotation(100, -90);
		chassis.stop();
		avanceLigne(1);
		chassis.openPliers();
		avance(-250);
		rotation(100,180);
		//avance(-250);
		avanceLigne(3);
		//travelTo(2500, 500);
		//travelTo(1500, 500);
		detecteObjet();
		attrapeObjet();
		travelTo(3000, 500);
		chassis.openPliers();
		avance(-250);
		chassis.closePliers();
	}

}



