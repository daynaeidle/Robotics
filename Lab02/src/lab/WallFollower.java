package lab;

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
		final float KpFront = 20;
		final float Ki = 2;
		final float Kd = (float) 30;
		final float SIDE_THRESHOLD = 50;
		final float FRONT_THRESHOLD = 35;

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
		float frontError = 0;
		float prevError = 0;

		buttons.waitForAnyPress();

		LEFT_MOTOR.backward();
		RIGHT_MOTOR.backward();

		while (true) {

			//fetch the samples
			sp1.fetchSample(sample1, 0);
			sp2.fetchSample(sample2, 0);

			//set the side distance
			if (sample1[0] * 100 >= SIDE_THRESHOLD) {
				sideDistance = SIDE_THRESHOLD;
			} else {
				sideDistance = sample1[0] * 100;
			}

			//set the front distance
			if (sample2[0] * 100 >= SIDE_THRESHOLD) {
				frontDistance = SIDE_THRESHOLD;
			} else {
				frontDistance = sample2[0] * 100;
			}

			//print out the values
			LCD.drawString(String.valueOf(sideDistance), 0, 1);
			LCD.drawString(String.valueOf(frontDistance), 0, 2);

			//check where the robot is in relation to the walls
			if (frontDistance >= FRONT_THRESHOLD && sideDistance >= SIDE_THRESHOLD) {
				frontError = GOAL_DISTANCE - frontDistance;
				
				//turn clockwise to find a wall
				RIGHT_MOTOR.setSpeed(DEFAULT_SPEED);
				LEFT_MOTOR.setSpeed(DEFAULT_SPEED + ((KpFront * frontError) * -1));
				LCD.drawString("BACK", 0, 4);
				
				
			} else if (frontDistance >= FRONT_THRESHOLD) {
				
				//straighten out on the wall
				LCD.drawString("FORWARD", 0, 4);

				pError = GOAL_DISTANCE - sideDistance;

				LCD.drawString("pError: " + String.valueOf(pError), 0, 3);

				integral = updateIntegral(integral, pError);

				//calculate new speed based on PID constants
				float desiredSpeed = calculateSpeed(Kp, Ki, Kd, pError, prevError, integral);
				
				if (pError < 0) {

					LEFT_MOTOR.setSpeed(DEFAULT_SPEED + desiredSpeed * -1);
					RIGHT_MOTOR.setSpeed(DEFAULT_SPEED);

				} else if (pError > 0) {

					RIGHT_MOTOR.setSpeed(DEFAULT_SPEED + desiredSpeed);
					LEFT_MOTOR.setSpeed(DEFAULT_SPEED);

				} else if (pError == 0) {
					RIGHT_MOTOR.setSpeed(DEFAULT_SPEED);
					LEFT_MOTOR.setSpeed(DEFAULT_SPEED);
				}

				prevError = pError;

			} else {
				//turn away from the wall
				frontError = GOAL_DISTANCE - frontDistance;
				
				if (frontError < 0) {
					if ((KpFront * frontError) * -1 <= 0)LEFT_MOTOR.setSpeed(50);
					else LEFT_MOTOR.setSpeed(KpFront * frontError * -1);
					RIGHT_MOTOR.setSpeed(DEFAULT_SPEED + ((KpFront * frontError) * -1));
					
				}else if (frontError >= 0) {
					if (KpFront * frontError <= 0)LEFT_MOTOR.setSpeed(50);
					else LEFT_MOTOR.setSpeed(KpFront * frontError);
					RIGHT_MOTOR.setSpeed(DEFAULT_SPEED + (KpFront * frontError));
					
				}
				
				LCD.drawString("TURNING", 0, 4);
			}

		}

	}

	/**
	 * Calculate speed based on error and PID constants
	 * @param kp
	 * @param ki
	 * @param kd
	 * @param error
	 * @param prevError
	 * @param integral
	 * @return float
	 */
	static float calculateSpeed(float kp, float ki, float kd, float error, float prevError, float integral) {

		float p = kp * error;

		float i = ki * integral;

		float d = kd * updateDerivative(error, prevError);

		return p + i + d;

	}

	/**
	 * Update the integral based on the current error
	 * @param integral
	 * @param error
	 * @return float
	 */
	static float updateIntegral(float integral, float error) {
		float newIntegral = integral + error;
		if (newIntegral > 20) {
			return 20;
		} else if (newIntegral < -20) {
			return -20;
		} else {
			return newIntegral;
		}

	}

	/**
	 * Update the derivative based on error
	 * @param error
	 * @param prevError
	 * @return float
	 */
	static float updateDerivative(float error, float prevError) {
		return prevError - error;
	}

}
