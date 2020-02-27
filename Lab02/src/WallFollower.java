import lejos.hardware.BrickFinder;
import lejos.hardware.Keys;
import lejos.hardware.ev3.EV3;
import lejos.hardware.lcd.LCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.MotorPort;
import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.NXTUltrasonicSensor;
import lejos.robotics.SampleProvider;

public class WallFollower {

	public static void main(String[] args) {
		
		final int DEFAULT_SPEED = 360;
		final float GOAL_DISTANCE = 25;
		final float Kp = 15;
		final float THRESHOLD = 50;
		
		float distance;
		
		EV3LargeRegulatedMotor LEFT_MOTOR = new EV3LargeRegulatedMotor(MotorPort.A);
		EV3LargeRegulatedMotor RIGHT_MOTOR = new EV3LargeRegulatedMotor(MotorPort.D);
	
		EV3 ev3brick = (EV3) BrickFinder.getLocal();
		
		NXTUltrasonicSensor sideSensor = new NXTUltrasonicSensor(SensorPort.S1);
		
		Keys buttons = ev3brick.getKeys();
		
		SampleProvider sp1 = sideSensor.getDistanceMode();
		
		int sampleSize1 = sp1.sampleSize();
		float[] sample1 = new float[sampleSize1];
		
		float pError = 0;
		
		LEFT_MOTOR.backward();
		RIGHT_MOTOR.backward();
		
		while(true) {
			
			sp1.fetchSample(sample1, 0);
			LCD.drawString(String.valueOf(sample1[0] * 100), 1, 0);
			
			if (sample1[0] * 100 >= THRESHOLD) {
				distance = THRESHOLD;
			}else {
				distance = sample1[0] * 100;
			}
			
			pError = GOAL_DISTANCE - distance;
			
			LCD.drawString("pError: " + String.valueOf(pError), 1, 2);
			
			
			
			if (pError < 0) {
				
				LEFT_MOTOR.setSpeed(DEFAULT_SPEED + ((Kp * -1) * pError));
				RIGHT_MOTOR.setSpeed(DEFAULT_SPEED);
				
			}else if (pError > 0) {
				
				RIGHT_MOTOR.setSpeed(DEFAULT_SPEED + (Kp * pError));
				LEFT_MOTOR.setSpeed(DEFAULT_SPEED);
				
			}else if (pError == 0) {
				RIGHT_MOTOR.setSpeed(DEFAULT_SPEED);
				LEFT_MOTOR.setSpeed(DEFAULT_SPEED);
			}
			
			//if error is negative, increase speed of the right motor
			//if error is positive, increase the speed of the left motor

		}

	}

}
