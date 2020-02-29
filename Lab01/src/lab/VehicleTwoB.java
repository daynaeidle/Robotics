package lab;

import lejos.hardware.Keys;
import lejos.hardware.BrickFinder;
import lejos.hardware.ev3.EV3;
import lejos.hardware.lcd.LCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.MotorPort;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.port.SensorPort;
import lejos.robotics.SampleProvider;

public class VehicleTwoB {

	public static void main(String[] args) {

		final double LIGHT_THRESHOLD = 0.25;

		EV3LargeRegulatedMotor LEFT_MOTOR = new EV3LargeRegulatedMotor(MotorPort.A);
		EV3LargeRegulatedMotor RIGHT_MOTOR = new EV3LargeRegulatedMotor(MotorPort.D);

		EV3ColorSensor lightSensor1 = new EV3ColorSensor(SensorPort.S4);
		EV3ColorSensor lightSensor2 = new EV3ColorSensor(SensorPort.S1);

		EV3 ev3brick = (EV3) BrickFinder.getLocal();

		Keys buttons = ev3brick.getKeys();

		SampleProvider sp1 = lightSensor1.getAmbientMode();

		int sampleSize1 = sp1.sampleSize();
		float[] sample1 = new float[sampleSize1];

		SampleProvider sp2 = lightSensor2.getAmbientMode();

		int sampleSize2 = sp2.sampleSize();
		float[] sample2 = new float[sampleSize2];

		buttons.waitForAnyPress();

		LEFT_MOTOR.backward();
		RIGHT_MOTOR.backward();

		while (true) {

			sp1.fetchSample(sample1, 0);
			LCD.drawString(String.valueOf(sample1[0]), 1, 0);

			sp2.fetchSample(sample2, 0);
			LCD.drawString(String.valueOf(sample2[0]), 1, 1);

			if (sample1[0] < LIGHT_THRESHOLD && sample2[0] < LIGHT_THRESHOLD) {
				LEFT_MOTOR.backward();
				RIGHT_MOTOR.backward();
			} else if (sample1[0] > LIGHT_THRESHOLD && sample2[0] > LIGHT_THRESHOLD) {
				if (sample1[0] > sample2[0]) {
					LEFT_MOTOR.backward();
					RIGHT_MOTOR.stop();
				} else if (sample2[0] > sample1[0]) {
					RIGHT_MOTOR.backward();
					LEFT_MOTOR.stop();
				}
			} else if (sample1[0] > LIGHT_THRESHOLD) {
				LEFT_MOTOR.backward();
				RIGHT_MOTOR.stop();
			} else if (sample2[0] > LIGHT_THRESHOLD) {
				RIGHT_MOTOR.backward();
				LEFT_MOTOR.stop();
			}

		}
	}
}
