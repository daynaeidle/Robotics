import lejos.hardware.sensor.NXTUltrasonicSensor;
import lejos.robotics.SampleProvider;

public class PIDController {
	private float kp, ki, kd, currError, prevError, currDistance, proportion, integral, derivative;
	private int sampleSize;
	private float[] sample;
	private SampleProvider sp;
	private NXTUltrasonicSensor sensor;
	private final float GOAL_DISTANCE = 25;
	
	PIDController(){
		this.kp = 0;
		this.ki = 0;
		this.kd = 0;
		this.integral = 0;
		this.sampleSize = 0;
		this.sample = null;
		this.sensor = null;
		this.sp = null;
		this.currError = 0;
	}
	
	/**
	 * PIDController
	 * 
	 * This constructor takes a sensor, and the constants being use on the PID controller. The
	 * Constructor will contain all of the logic necessary for computing samples, distance, and
	 * error.
	 * 
	 * @param sensor
	 * @param kp
	 * @param ki
	 * @param kd
	 */
	PIDController(NXTUltrasonicSensor sensor, float kp, float ki, float kd){
		this.sensor = sensor;
		this.kp = kp;
		this.ki = ki;
		this.kd = kd;
		this.integral = 0;
		this.currDistance = 0;
		this.currError = 0;
		this.sp = this.sensor.getDistanceMode();
		this.sampleSize = sp.sampleSize();
		this.sample = new float[sampleSize];
	}
	
	
	/**
	 * fetchSample
	 * 
	 * This method gets the current sample from the sensor, and it converts
	 * it to a distance in cm. This method is private because using it in
	 * a different class could result in unsynchronized advances of the sample.
	 */
	private void fetchSample() {
		this.sp.fetchSample(this.sample, 0);
		this.currDistance = sample[0] * 100;
	}
	
	/**
	 * getError
	 * 
	 * Gets the magnitude and direction of the current state of error within the system.
	 * This method is also used to advance the state of the current sample
	 * @return The amount of error in the system's current state
	 */
	private float getError() {
		fetchSample();
		this.prevError = this.currError;
		this.currError = GOAL_DISTANCE - this.currDistance;
		// We could put a cap on the current error here
		return this.currError;
	}
	
	/**
	 * computePID
	 * 
	 * Calculates the amount of speed to be applied to the current system using 
	 * proportions, integrals, and derivatives
	 * 
	 * @return Amount of speed to added to the motor
	 */
	public float computePID() {
		getError();
		// P will be the amount of current error
		this.proportion = this.currError;
		
		// I will be the sum of all current error in the system. Limit is set at 50 (needs to be tuned)
		if (this.integral < 50 && this.integral > -50) {
			this.integral += this.currError;
		} else if (this.integral > 50) {
			this.integral = 50;
		} else if (this.integral < -50) {
			this.integral = -50;
		}
		// D will be the amount of change between the error last detected and the current error
		this.derivative = prevError - currError;
		return (this.proportion * kp) + (this.integral * ki) + (this.derivative * kd);
	}
	
	public float getCurrError() {
		return this.currError;
	}
	
	public float getCurrDistance() {
		return this.currDistance;
	}
	
}
