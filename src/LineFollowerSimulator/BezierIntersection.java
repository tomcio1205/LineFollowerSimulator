package LineFollowerSimulator;

import java.awt.geom.Line2D;
import java.awt.geom.Point2D;
import java.awt.geom.CubicCurve2D;
import java.awt.geom.GeneralPath;
import java.awt.geom.PathIterator;
import java.util.ArrayList;

/**
 * Static class calculates intersections of a line segment and Bezier curve.
 * Only intersection with cubic bezier curves are implemented so far.
 * @author Ondrej Stanek
 */
public class BezierIntersection {

    // TODO Possible optimalization: Benefit from the fact that the convex hull of the Bézier polygon contains the Bézier curve.
    static final double EPSILON = 1E-5;

    /**
     * Calculates intersection points of a line segment and GeneralPath object.
     * GeneralPath object must contain only cubic bezier curves, other geometric shapes are not supportet yet
     *
     * @return List of intersection points of path and line
     */
    public static ArrayList<Point2D> GetIntersections(GeneralPath path, Line2D line) {

	ArrayList<Point2D> intersections = new ArrayList<Point2D>();

	PathIterator iterator = path.getPathIterator(null);
	double[] coords = new double[6];
	Point2D lastPoint = new Point2D.Double();
	while (!iterator.isDone()) {
	    int type = iterator.currentSegment(coords);
	    switch (type) {
		case PathIterator.SEG_MOVETO:
		    lastPoint.setLocation(coords[0], coords[1]);
		    break;
		case PathIterator.SEG_CUBICTO:
		    CubicCurve2D curve = new CubicCurve2D.Double(lastPoint.getX(), lastPoint.getY(), coords[0], coords[1], coords[2], coords[3], coords[4], coords[5]);
		    intersections.addAll(GetIntersections(curve, line));
		    lastPoint.setLocation(coords[4], coords[5]);
		    break;
		case PathIterator.SEG_QUADTO:
		    throw new UnsupportedOperationException("Quadratic curves intersections not implemented yet!");
		case PathIterator.SEG_LINETO:
		    throw new UnsupportedOperationException("Lines intersections not implemented yet!");
		default:
		    throw new UnsupportedOperationException("Type " + type + " intersections not implemented yet!");
	    }
	    iterator.next();
	}
	return intersections;
    }

    /**
     * Calculates intersection points of a line segment and cubic Bezier curve
     *
     * @return List of intersection points
     */
    public static ArrayList<Point2D> GetIntersections(CubicCurve2D curve, Line2D line) {
	double[] y = new double[4];
        //wspolrzedna y pocztaku segmentu 
	y[0] = curve.getY1();
        //wspolrzedna y 1-go pkt kontrolnego
	y[1] = curve.getCtrlY1();
        //wspolrzedna y 2-go pkt kontrolnego
	y[2] = curve.getCtrlY2();
        //wspolrzedna y konca segmentu
	y[3] = curve.getY2();

	double[] x = new double[4];
        //wspolrzedna x pocztaku segmentu 
 	x[0] = curve.getX1();
        //wspolrzedna x 1-go pkt kontrolnego
	x[1] = curve.getCtrlX1();
        //wspolrzedna x 2-go pkt kontrolnego
	x[2] = curve.getCtrlX2();
        //wspolrzedna x konca segmentu
	x[3] = curve.getX2();

	// rotate the control points so that the line is an x axis

	// calculate intersection of the line and y axis
        //jesli linia z czujnikami jest prosotpadla do osi x to 
        //nie ma punktu przeciecia z osia y,
        // taka sytuacja wystepuje gdy x1-x2~=0
	if (Math.abs(line.getX1() - line.getX2()) <= EPSILON) { //there is no intersection with y axis
	    //switch x for y and do offset
            //zamieniamy x z y
	    double[] temp;
	    temp = y;
	    y = x;
	    x = temp;
	    for (int i = 0; i < y.length; i++) {
		y[i] -= line.getX1();
	    }
	} else { // calculate intersection point of the line and y axis
	    // y = bx + c
	    double b = (line.getY1() - line.getY2()) / (line.getX1() - line.getX2());
            //przeksztalacamy y-bx = c
	    double c = line.getY1() - b * line.getX1();
            //b to tangens kata stad kat to arc tangens (skad - ?)
            // cwiartka??
	    double angle = -Math.atan(b);

	    // calculate translation vector for line
	    double vect_x = -(b * c) / (b * b + 1);
	    double vect_y = c / (b * b + 1);

	    for (int i = 0; i < y.length; i++) {

		// do control point translation
		x[i] -= vect_x;
		y[i] -= vect_y;

		// do point transformation (rotation) - y coordinates only
		y[i] = x[i] * Math.sin(angle) + (y[i]) * Math.cos(angle);
	    }
	}
        //zapis macierzowy krzywej Beziera
	double[] coefs = new double[4];
	coefs[3] = -y[0] + 3 * y[1] - 3 * y[2] + y[3];
	coefs[2] = 3 * y[0] - 6 * y[1] + 3 * y[2];
	coefs[1] = -3 * y[0] + 3 * y[1];
	coefs[0] = y[0];

	int numRoots = CubicCurve2D.solveCubic(coefs); //find roots for cubic polynom

	ArrayList<Point2D> intersections = new ArrayList<Point2D>(numRoots);

	for (int i = 0; i < numRoots; i++) { //evaluate roots and compute intersection points
	    if (coefs[i] >= 0 && coefs[i] <= 1) {
		Point2D point = getPointOnCurve(curve, coefs[i]);
		if (pointLiesOnLine(point, line)) {
		    intersections.add(point);
		}
	    }
	}
	return intersections;
    }

    private static Point2D getPointOnCurve(CubicCurve2D curve, double t) {
	double x0 = curve.getX1();
	double x1 = curve.getCtrlX1();
	double x2 = curve.getCtrlX2();
	double x3 = curve.getX2();

	double y0 = curve.getY1();
	double y1 = curve.getCtrlY1();
	double y2 = curve.getCtrlY2();
	double y3 = curve.getY2();

	double x = (1 - t) * (1 - t) * (1 - t) * x0 + 3 * (1 - t) * (1 - t) * t * x1 + 3 * (1 - t) * t * t * x2 + t * t * t * x3;
	double y = (1 - t) * (1 - t) * (1 - t) * y0 + 3 * (1 - t) * (1 - t) * t * y1 + 3 * (1 - t) * t * t * y2 + t * t * t * y3;

	return new Point2D.Double(x, y);
    }

    private static boolean pointLiesOnLine(Point2D point, Line2D line) {
	if (line.ptSegDist(point) <= EPSILON) {
	    return true;
	} else {
	    return false;
	}
    }
}
