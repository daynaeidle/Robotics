package lab;

import lejos.hardware.Keys;
import lejos.hardware.BrickFinder;
import lejos.hardware.ev3.EV3;
import lejos.hardware.lcd.LCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.MotorPort;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.port.SensorPort;
import lejos.robotics.Color;
import lejos.hardware.sensor.BaseSensor;
import lejos.robotics.SampleProvider;
import lejos.utility.Delay;

public class VehicleTwoA {

	public static void main(String[] args) {
		
		EV3LargeRegulatedMotor LEFT_MOTOR = new EV3LargeRegulatedMotor(MotorPort.A);
		EV3LargeRegulatedMotor RIGHT_MOTOR = new EV3LargeRegulatedMotor(MotorPort.B);
		
		EV3ColorSensor lightSensor1 = new EV3ColorSensor(SensorPort.S1);
		
		SampleProvider sp1 = lightSensor1.getAmbientMode();

		int sampleSize1 = sp1.sampleSize();
		float[] sample1 = new float[sampleSize1];
		
		LEFT_MOTOR.forward();
		RIGHT_MOTOR.forward();
		
		while (true) {
			
			sp1.fetchSample(sample1, 0);
			LCD.drawString(String.valueOf(sample1[0]), 4, 0);
			
			LEFT_MOTOR.setSpeed((int)mapRange(sample1[0]) + 10);
			LEFT_MOTOR.forward();
			
		}

	}
	
	//from Rosetta Code
	public static double mapRange(double mapVal){
		return 0.0 + ((mapVal - 0.0)*(100.0 - 0.0))/(1.0 - 0.0);
	}

}
