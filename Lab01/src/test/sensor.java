package test;

import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.port.SensorPort;
import lejos.robotics.Color;
import lejos.hardware.BrickFinder;
import lejos.hardware.Keys;
import lejos.hardware.ev3.EV3;
import lejos.hardware.lcd.LCD;
import lejos.hardware.sensor.BaseSensor;
import lejos.robotics.SampleProvider;
import lejos.utility.Delay;

public class sensor {

	public static void main(String[] args) {
		// TODO Auto-generated method stub

		EV3ColorSensor lightSensor = new EV3ColorSensor(SensorPort.S1);

		EV3 ev3brick = (EV3) BrickFinder.getLocal();

		Keys buttons = ev3brick.getKeys();

		// lightSensor.setFloodlight(Color.NONE);

		SampleProvider sp = lightSensor.getAmbientMode();

		int sampleSize = sp.sampleSize();
		float[] sample = new float[sampleSize];

		while (true) {
			sp.fetchSample(sample, 0);
			LCD.drawString(String.valueOf(sample[0]), 4, 0);
		}

		// Takes some samples and prints them
		/*
		 * for (int i = 0; i < 10; i++) {
		 * 
		 * System.out.println("N=" + i + " Sample={}" + (int)sample[0]);
		 * //LCD.drawString(str, x, y); Delay.msDelay(1000); }
		 */

	}

}
