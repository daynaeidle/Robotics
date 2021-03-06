package lab;

import lejos.hardware.Keys;
import lejos.hardware.BrickFinder;
import lejos.hardware.ev3.EV3;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.MotorPort;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.port.SensorPort;
import lejos.robotics.SampleProvider;

public class VehicleThreeA {

	public static void main(String[] args) {
		
		final double LIGHT_THRESHOLD = 0.15;
		final double TURNING_MAX_LIGHT = 0.5;
		final double MAX_LIGHT_VALUE = 0.85;
		
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
			//LCD.drawString(String.valueOf(sample1[0]), 1, 0);
			
			sp2.fetchSample(sample2, 0);
			//LCD.drawString(String.valueOf(sample2[0]), 1, 1);
			
			//if neither motor detects a lot of light, set the speeds according to the light value
			if (sample1[0] < LIGHT_THRESHOLD && sample2[0] < LIGHT_THRESHOLD) {
				LEFT_MOTOR.setSpeed(getSpeedFromLight(sample1[0]));
				RIGHT_MOTOR.setSpeed(getSpeedFromLight(sample1[0]));
				LEFT_MOTOR.backward();
				RIGHT_MOTOR.backward();
			}else if (sample1[0] > MAX_LIGHT_VALUE || sample2[0] > MAX_LIGHT_VALUE) {
				//if either sensor reads more than the max light value, stop both motors
				LEFT_MOTOR.stop();
				RIGHT_MOTOR.stop();
			} else if (sample1[0] > LIGHT_THRESHOLD && sample2[0] > LIGHT_THRESHOLD) {
				//if both detect a lot of light...
			
				if (sample1[0] > sample2[0]) {
					
					//if left detects more than right
					System.out.println("Light detected on S1");
					System.out.println("Light S1: " + sample1[0]);

					//turn in that direction if it's still decently far away
					if (sample1[0] < TURNING_MAX_LIGHT) {
						LEFT_MOTOR.backward();
						RIGHT_MOTOR.stop();	
					}
					
					//lower the speed as it gets closer
					LEFT_MOTOR.setSpeed(getSpeedFromLight(sample1[0]));
					RIGHT_MOTOR.setSpeed(getSpeedFromLight(sample1[0]));
					System.out.println("Speed LEFT: " + getSpeedFromLight(sample1[0]));
					LEFT_MOTOR.backward();
					RIGHT_MOTOR.backward();
				}else if (sample2[0] > sample1[0]) {
					
					//if right detects more than left
					System.out.println("Light detected on S2");
					System.out.println("Light S2: " + sample2[0]);
					//turn in that direction if it's still decently far away
					if (sample2[0] < TURNING_MAX_LIGHT) {
						RIGHT_MOTOR.backward();
						LEFT_MOTOR.stop();
					}
					
					//lower the speed as it gets closer
					RIGHT_MOTOR.setSpeed(getSpeedFromLight(sample2[0]));
					LEFT_MOTOR.setSpeed(getSpeedFromLight(sample2[0]));
					System.out.println("Speed RIGHT: " + getSpeedFromLight(sample2[0]));
					RIGHT_MOTOR.backward();
					LEFT_MOTOR.backward();
				}
			}else if (sample1[0] > LIGHT_THRESHOLD) {
				//if left detects more than right
				System.out.println("Light detected on S1");
				System.out.println("Light S1: " + sample1[0]);
				
				//turn in that direction if it's still decently far away
				if (sample1[0] < TURNING_MAX_LIGHT) {
					LEFT_MOTOR.backward();
					RIGHT_MOTOR.stop();	
				}
				//lower the speed as it gets closer
				LEFT_MOTOR.setSpeed(getSpeedFromLight(sample1[0]));
				RIGHT_MOTOR.setSpeed(getSpeedFromLight(sample1[0]));
				System.out.println("Speed LEFT: " + getSpeedFromLight(sample1[0]));
				LEFT_MOTOR.backward();
				RIGHT_MOTOR.backward();
			} else if (sample2[0] > LIGHT_THRESHOLD){
				
				//if right detects more than left
				System.out.println("Light detected on S2");
				System.out.println("Light S2: " + sample2[0]);
				
				//turn in that direction if it's still decently far away
				if (sample2[0] < TURNING_MAX_LIGHT) {
					RIGHT_MOTOR.backward();
					LEFT_MOTOR.stop();
				}
				
				//lower the speed as it gets closer
				RIGHT_MOTOR.setSpeed(getSpeedFromLight(sample2[0]));
				LEFT_MOTOR.setSpeed(getSpeedFromLight(sample2[0]));
				System.out.println("Speed RIGHT: " + getSpeedFromLight(sample2[0]));
				RIGHT_MOTOR.backward();
				LEFT_MOTOR.backward();
			}
			
		}
	}
	
	public static int getSpeedFromLight(double lightVal) {
		return 100 - (int)(lightVal * 100);
	}
}
