package LineFollowerSimulator;
import static java.lang.System.out;
/**
 * Simulation of a PID regulator for a line following robot.
 * The objects creates its own thread that periodicaly scans for line position
 * and updates motor speeds.
 *
 * @author Ondrej Stanek
 */
public class PIDregulator implements Runnable {

    private volatile double p;
    private volatile double i;
    private volatile double d;
    private volatile double speed;
    private volatile long delay;
    private volatile double previousLinePosition;
    private volatile double sumLinePositions;
    private MotorController motorController;
    private Sensor sensor;
    private Thread timer;
    

    /**
     * Initializes PID regulator.
     *
     * @param motorCtrl a MotorController that will simulate robot movement
     * @param sensor sensor that will provide line position
     */
    public PIDregulator(MotorController motorCtrl, Sensor sensor) {
	this.motorController = motorCtrl;
	this.sensor = sensor;

	timer = new Thread(this);
    }

    /**
     * Runs a thread that periodically obtain line position from a sensor and updates motors' speed.
     */
    public void StartPIDregulation() {
	timer.start();
    }

    /**
     * Set the refresh frequency for PID regulation, e.g. how often the speed of motors is updated.
     * @param freq  refresh frequency in Hz
     */
    public void SetFrequency(int freq) {
	delay = 1000 / freq;
    }

    /**
     * Thread that periodicaly updates motors' speed acording to the line position
     * contains the PID regulator loop itself
     */
    public void run() {
	// Remember the starting time
	long tm = System.currentTimeMillis();
	while (Thread.currentThread() == timer) {

	    double position = sensor.GetLinePosition();

	    if (Math.abs(position) > 1) { // if robot is off line, start turning
		if (position < 0) {
		    motorController.setSpeed(0, speed);
		} else {
		    motorController.setSpeed(speed, 0);
		}
		sumLinePositions = 0;
	    } else {

		double drive = p * position + i * sumLinePositions + d * (position - previousLinePosition);
		drive = drive * speed;


		double l, r;

		if (drive < 0) {
		    l = Math.min(-drive, speed);
		    r = speed;
		} else {
		    l = speed;
		    r = Math.min(drive, speed);
		}
		motorController.setSpeed(l, r);

		previousLinePosition = position;
		sumLinePositions += position;
	    }

	    try {
		tm += delay;
		Thread.sleep(Math.max(0, tm - System.currentTimeMillis()));
                out.println(System.currentTimeMillis());
	    } catch (InterruptedException e) {
		break;
	    }
	}
    }

    /**
     * Sets the proportional factor of PID regulator, if 0 is set,
     * the proportional regulation is supressed.
     *
     * @param p positive number
     */
    public void setP(double p) {
	this.p = p;
    }

    /**
     * Sets the integral factor of PID regulator, if 0 is set,
     * the integral regulation is supressed.
     *
     * @param i positive number
     */
    public void setI(double i) {
	this.i = i;
    }

    /**
     * Sets the derivate factor of PID regulator, if 0 is set,
     * the diferencial regulation is supressed.
     *
     * @param d positive number
     */
    public void setD(double d) {
	this.d = d;
    }

    /**
     * Sets the maximum speed of the robot.
     * @param speed positive number
     */
    public void setSpeed(double speed) {
	this.speed = speed;
    }
}

