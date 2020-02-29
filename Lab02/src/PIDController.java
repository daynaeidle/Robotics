import lejos.hardware.sensor.NXTUltrasonicSensor;
import lejos.robotics.SampleProvider;

public class PIDController {
	private float kp, ki, kd, currError, prevError, integral, currDistance;
	private int sampleSize;
	private float[] sample;
	private SampleProvider sp;
	private NXTUltrasonicSensor sensor;
	private final int DEFAULT_SPEED = 360;
	private final float GOAL_DISTANCE = 25;
	
	PIDController(){
		this.kp = 0;
		this.ki = 0;
		this.kd = 0;
		this.integral = 0;
	}
	
	PIDController(NXTUltrasonicSensor sensor, float kp, float ki, float kd){
		this.sensor = sensor;
		this.kp = kp;
		this.ki = ki;
		this.kd = kd;
		this.integral = 0;
		this.currDistance = 0;
		this.sp = this.sensor.getDistanceMode();
		this.sampleSize = sp.sampleSize();
		this.sample = new float[sampleSize];
	}
	
	private float fetchSample() {
		this.sp.fetchSample(this.sample, 0);
		return sample[0];
	}
	
	/**
	 * getError
	 * 
	 * Gets the magnitude and direction of the current state of error within the system.
	 * @return The amount of error in the system's current state
	 */
	public float getError() {
		this.prevError = this.currError;
		this.currDistance = fetchSample();
		return GOAL_DISTANCE - this.currDistance;
	}
	
	public float computePID() {
		getError();
		float proportion = this.currError;
		this.integral += this.currError;
		float derivative = currError - prevError;
		return (proportion * kp) + (this.integral * ki) + (derivative * kd);
	}
	
	

	public float getKp() {
		return kp;
	}

	public void setKp(float kp) {
		this.kp = kp;
	}

	public float getKi() {
		return ki;
	}

	public void setKi(float ki) {
		this.ki = ki;
	}

	public float getKd() {
		return kd;
	}

	public void setKd(float kd) {
		this.kd = kd;
	}
	
	
}
