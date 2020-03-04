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
		final float Kp = 50;
		final float Ki = 2;
		final float Kd = (float)30;
		final float SIDE_THRESHOLD = 50;
		final float FRONT_THRESHOLD = 25;
		
		/**
		 * Numbers we like
		 * Kp = 50
		 * Ki = 2
		 * Kd = 30
		 */
		
		float sideDistance;
		float frontDistance;
		float integral = 0;
		
		EV3LargeRegulatedMotor LEFT_MOTOR = new EV3LargeRegulatedMotor(MotorPort.A);
		EV3LargeRegulatedMotor RIGHT_MOTOR = new EV3LargeRegulatedMotor(MotorPort.D);
	
		EV3 ev3brick = (EV3) BrickFinder.getLocal();
		
		NXTUltrasonicSensor sideSensor = new NXTUltrasonicSensor(SensorPort.S1);
		NXTUltrasonicSensor frontSensor = new NXTUltrasonicSensor(SensorPort.S4);
		
		Keys buttons = ev3brick.getKeys();
		
		SampleProvider sp1 = sideSensor.getDistanceMode();
		SampleProvider sp2 = frontSensor.getDistanceMode();
		
		int sampleSize1 = sp1.sampleSize();
		float[] sample1 = new float[sampleSize1];
		
		int sampleSize2 = sp2.sampleSize();
		float[] sample2 = new float[sampleSize2];
		
		float pError = 0;
		float prevError = 0;
		
		buttons.waitForAnyPress();
		
		LEFT_MOTOR.backward();
		RIGHT_MOTOR.backward();
		
		while(true) {
			
			sp1.fetchSample(sample1, 0);
			
			
			sp2.fetchSample(sample2, 0);
			
		
			
			if (sample1[0] * 100 >= SIDE_THRESHOLD) {
				sideDistance = SIDE_THRESHOLD;
			}else {
				sideDistance = sample1[0] * 100;
			}
			
			if (sample2[0] * 100 >= SIDE_THRESHOLD) {
				frontDistance = SIDE_THRESHOLD;
			}else {
				frontDistance = sample2[0] * 100;
			}
			
			LCD.drawString(String.valueOf(sideDistance), 0, 1);
			LCD.drawString(String.valueOf(frontDistance), 0, 2);
			
			
			if (frontDistance >= FRONT_THRESHOLD) {
				LCD.drawString("FORWARD", 0, 4);
				
				pError = GOAL_DISTANCE - sideDistance;
				
				LCD.drawString("pError: " + String.valueOf(pError), 0, 3);
				
				integral = updateIntegral(integral, pError);
				
				
				
				float desiredSpeed = calculateSpeed(Kp, Ki, Kd, pError, prevError, integral);
				if (pError < 0) {
					
					LEFT_MOTOR.setSpeed(DEFAULT_SPEED + desiredSpeed * -1);
					RIGHT_MOTOR.setSpeed(DEFAULT_SPEED);
					
				}else if (pError > 0) {
					
					RIGHT_MOTOR.setSpeed(DEFAULT_SPEED + desiredSpeed);
					LEFT_MOTOR.setSpeed(DEFAULT_SPEED);
					
				}else if (pError == 0) {
					RIGHT_MOTOR.setSpeed(DEFAULT_SPEED);
					LEFT_MOTOR.setSpeed(DEFAULT_SPEED);
				}
			
				prevError = pError;
				
			}else {
				LEFT_MOTOR.setSpeed(1);
				RIGHT_MOTOR.setSpeed(DEFAULT_SPEED + 500);
				LCD.drawString("TURNING", 0, 4);
			}
			
			

		}

	}
	
	static float calculateSpeed(float kp, float ki, float kd, float error, float prevError, float integral) {
		
		float p = kp * error;
		
		float i = ki * integral;
		
		float d = kd * updateDerivative(error, prevError);
		
		return p + i + d;
		
	}
	
	static float updateIntegral(float integral, float error) {
		float newIntegral = integral + error;
		if(newIntegral > 20) {
			return 20;
		}else if ( newIntegral < -20){
			return -20;
		}else {
			return newIntegral;
		}
		
	}
	
	static float updateDerivative(float error, float prevError) {
		return prevError - error;
	}

}
