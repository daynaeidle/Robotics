package test;

import lejos.hardware.Keys;
import lejos.hardware.BrickFinder;
import lejos.hardware.ev3.EV3;
import lejos.hardware.lcd.LCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.MotorPort;

public class motor {

	public static void main(String[] args) {
		
		EV3LargeRegulatedMotor LEFT_MOTOR = new EV3LargeRegulatedMotor(MotorPort.A);
		EV3LargeRegulatedMotor RIGHT_MOTOR = new EV3LargeRegulatedMotor(MotorPort.D);
		
		EV3 ev3brick = (EV3) BrickFinder.getLocal();
		
		Keys buttons = ev3brick.getKeys();
		
		buttons.waitForAnyPress();
		
		LEFT_MOTOR.forward();
		RIGHT_MOTOR.forward();
		LCD.drawString("Forward", 0, 0);
		
		buttons.waitForAnyPress();
		
		LEFT_MOTOR.setSpeed(0);
		RIGHT_MOTOR.setSpeed(0);
		LEFT_MOTOR.forward();
		RIGHT_MOTOR.forward();
		
		buttons.waitForAnyPress();
		
		LEFT_MOTOR.backward();
		RIGHT_MOTOR.backward();
		LCD.drawString("Backward", 0, 1);
		
		buttons.waitForAnyPress();
		
		LEFT_MOTOR.forward();
		RIGHT_MOTOR.stop();
		LCD.drawString("Left only", 0, 2);
		
		buttons.waitForAnyPress();
		
		LEFT_MOTOR.stop();
		RIGHT_MOTOR.stop();
		LCD.drawString("Stop", 0, 3);
		

	}

}
