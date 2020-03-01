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
		final float Kp = 100;
		final float Ki = 1;
		final float Kd = (float) 1;
		final float THRESHOLD = 50;

		EV3LargeRegulatedMotor LEFT_MOTOR = new EV3LargeRegulatedMotor(MotorPort.A);
		EV3LargeRegulatedMotor RIGHT_MOTOR = new EV3LargeRegulatedMotor(MotorPort.D);

		EV3 ev3brick = (EV3) BrickFinder.getLocal();

		NXTUltrasonicSensor sideSensor = new NXTUltrasonicSensor(SensorPort.S1);

		Keys buttons = ev3brick.getKeys();

		SampleProvider sp1 = sideSensor.getDistanceMode();

		int sampleSize1 = sp1.sampleSize();
		float[] sample1 = new float[sampleSize1];

		PIDController controller = new PIDController(sideSensor, Kp, Ki, Kd);
		buttons.waitForAnyPress();
		LEFT_MOTOR.backward();
		RIGHT_MOTOR.backward();
		
		while (true) {

			LCD.drawString("Error: " + String.valueOf(controller.getCurrError()), 1, 2);

			if (controller.getCurrError() < 0) {
				LEFT_MOTOR.setSpeed(DEFAULT_SPEED + (controller.computePID()) * -1);
				RIGHT_MOTOR.setSpeed(DEFAULT_SPEED);
			} else if (controller.getCurrError() > 0) {

				RIGHT_MOTOR.setSpeed((DEFAULT_SPEED + (controller.computePID())));
				LEFT_MOTOR.setSpeed(DEFAULT_SPEED);

			} else if (controller.getCurrError() == 0) {
				RIGHT_MOTOR.setSpeed(DEFAULT_SPEED);
				LEFT_MOTOR.setSpeed(DEFAULT_SPEED);
				controller.computePID();
			}
		}
	}
}
