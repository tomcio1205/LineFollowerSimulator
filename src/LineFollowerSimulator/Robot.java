package LineFollowerSimulator;

import java.awt.geom.AffineTransform;
import java.awt.geom.GeneralPath;
import java.awt.geom.Line2D;
import java.awt.geom.Point2D;
import java.awt.geom.Rectangle2D;
import java.util.ArrayList;
import java.util.Iterator;


 // klasa robot dziedziczy po generalpath
public class Robot extends GeneralPath.Double {
     // obroty robota w radianach
    protected volatile double angle = 0;
    // wspolrzedna x srodka obrotu robota
    protected volatile double x = 0;
     //wspolrzedna y .....
    protected volatile double y = 0;
    // domyslne wartosci
    private double defaultAngle = 0;
    private double defaultX = 0;
    private double defaultY = 0;
     //sterowanie silnikiem
    protected MotorController motorController;
    private double robot_width = 20; //szerokosc robota
    private volatile double wheel_gauge; //rozstaw osi
    private volatile double robot_height; //wysokosc robota
     // linia z czujnikami
    public Sensor lineSensor;
    //ksztalt robota prostokat 2d
    private Rectangle2D bounds;
     // regulator pid
    public PIDregulator regulator;
    //metoda robor przyjmuje 4 parametry:
    // wspolrzedna x, wspolrzedna y
    // kąt obrotu
    // ścieżka za którą będzie podążał robot
    public Robot(double x, double y, double angle, GeneralPath path) {
  // wywołanie metody okreslajacej rozstaw osi z parametrem 40
	setWheelGauge(40);
  //konstruktor klasy Sensor
	lineSensor = new Sensor(path);
  // konstruktor klasy MotorController
	motorController = new MotorController();
  // konstruktor klasy PIDregulator
	regulator = new PIDregulator(motorController, lineSensor);

	this.defaultX = x;
	this.defaultY = y;
	this.defaultAngle = angle;
// wywolanie metody resetujacej pozycje
	ResetPosition();
    }

    /**
     * Reset robots coordinates and angle to the startup position
     */
    public final void ResetPosition() {
	x = defaultX;
	y = defaultY;
	angle = defaultAngle;
    }

    /**
     * Sets the distance between robot wheels
     * @param gauge distance in pixels
     */
// ustaw rozstaw osi
    public final void setWheelGauge(double gauge) {
	wheel_gauge = gauge;
	robot_height = gauge;
// Rectangle2D.souble(height rectangle, width rectangle, x coordinate, y coordinate)
	bounds = new Rectangle2D.Double(-robot_width / 2, -robot_height / 2, robot_width, robot_height);
    }

    /**
     * The simulation of robot movement is done step by step.
     * Each time this function is called, the new step of animation is computed.
     * The function shall be called periodicaly.
     */
// metoda symulujaca ruch robota
    public void Move() {
// aktualizuj predkosc kol
	motorController.Update();

	// calculate new robots position
  // oszacuj nowa pozycje robota
  // kąt, oraz przebyty dystans wyliczane są z wzorów matematycznych:
  //http://rossum.sourceforge.net/papers/DiffSteer/
	angle += (motorController.getLeftSpeed() - motorController.getRightSpeed()) / wheel_gauge;
	double distance = (motorController.getLeftSpeed() + motorController.getRightSpeed()) /2;
	x += distance * Math.cos(angle);
	y += distance * Math.sin(angle);

	// calculate sensor module position
	lineSensor.UpdatePosition(this);

	updateShape();
    }
// metoda odpowiedzialna za wizualizacje obrotów robota
    private void updateShape() {
	this.reset();
	this.append(bounds, false);
	AffineTransform transform = new AffineTransform();
	transform.rotate(angle);
	this.transform(transform);
	transform = new AffineTransform();
	transform.translate(x, y);
	this.transform(transform);
	this.append(lineSensor.sensor_line, false); //the line sensor visualization
    }
}

class Sensor {

    private double width;
    private double location;
    //przeciwprostokątna
    private double hypotenuse;
    private double angle;
    public Line2D sensor_line;
    private GeneralPath path;
    private int lastSeen = 1;

    public final void setGeomtery(double width, double location) {
        //odsuniecie czujnikow
	this.location = location;
        //szerokosc czujnikow
	this.width = width;
	hypotenuse = Math.sqrt((width / 2) * (width / 2) + location * location);
	angle = Math.atan((width / 2) / location);
    }

    public Sensor(GeneralPath path) {
	sensor_line = new Line2D.Double();
	this.path = path;
	setGeomtery(30, 20);
    }

    public void UpdatePosition(Robot robot) {
        // wyliczenie punktow krancowych linii z czujnikami 
        // oraz zmiana ich polozenia
	double a = robot.angle - angle;
	double x1 = robot.x + hypotenuse * Math.cos(a);
	double y1 = robot.y + hypotenuse * Math.sin(a);

	a = robot.angle + angle;
	double x2 = robot.x + hypotenuse * Math.cos(a);
	double y2 = robot.y + hypotenuse * Math.sin(a);

	sensor_line.setLine(x1, y1, x2, y2);
    }

    public double GetLinePosition() {
        // wczytaj linie stanowiaca czujniki oraz sciezke 
	ArrayList<Point2D> intersections = BezierIntersection.GetIntersections(path, sensor_line);

	// select the closesst value to center
	double minDistance = java.lang.Double.POSITIVE_INFINITY;
	Iterator iterator = intersections.iterator();
	while (iterator.hasNext()) {
	    Point2D point = (Point2D) iterator.next();
	    double distance = point.distance(sensor_line.getX1(), sensor_line.getY1());
	    distance -= this.width / 2;
	    if (Math.abs(distance) < Math.abs(minDistance)) {
		minDistance = distance;
	    }
	}

	// if there is no line, adjust the sign of infinity depending on side where the line was seen last
	if (minDistance == java.lang.Double.POSITIVE_INFINITY) {
	    minDistance *= lastSeen;
	} else {
	    minDistance = minDistance / (this.width / 2);
	    if (minDistance < 0) {
		lastSeen = -1;
	    } else {
		lastSeen = 1;
	    }

	}
	return minDistance;
    }
}

// "sterownik silnika"
class MotorController {

    private volatile double leftSpeed, desiredLeftSpeed;
    private volatile double rightSpeed, desiredRightSpeed;
    private volatile double acceleration;

    public void Update() {
	leftSpeed = compute(leftSpeed, desiredLeftSpeed);
	rightSpeed = compute(rightSpeed, desiredRightSpeed);
    }
// oszacuj predkosc kol
    private double compute(double speed, double desired) {
	if (Math.abs(speed - desired) <= acceleration) {
	    return desired;
	} else {
	    if (speed < desired) {
		return speed + acceleration;
	    } else {
		return speed - acceleration;
	    }
	}

    }

    public void setSpeed(double left, double right) {
	this.desiredLeftSpeed = left;
	this.desiredRightSpeed = right;
    }

    public double getLeftSpeed() {
	return leftSpeed;
    }

    public double getRightSpeed() {
	return rightSpeed;
    }

    public void setAcceleration(double acceleration) {
	this.acceleration = acceleration;
    }
}
