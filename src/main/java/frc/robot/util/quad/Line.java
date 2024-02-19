package frc.robot.util.quad; 

public class Line {
    public double slope, intercept;
    public OrderedPair pointA, pointB;

    public Line(OrderedPair pointA, OrderedPair pointB) {
        this.slope = pointA.getSlopeBetweenPoints(pointB);
        this.intercept = pointA.getInterceptBetweenPoints(pointB);
        this.pointA = pointA;
        this.pointB = pointB;
    }

    public double getSlope() {
        return slope;
    }

    public double getIntercept() {
        return intercept;
    }

    public double valueAtX(double x) {
        if (getSlope() == Double.POSITIVE_INFINITY) {
            return Double.POSITIVE_INFINITY;
        }
        return x * slope + intercept;
    }

    public boolean isPointAboveLine(OrderedPair comparisonPoint) {
        if (getSlope() == Double.POSITIVE_INFINITY) {
            return (pointB.getX() <= comparisonPoint.getX());
        }
        return valueAtX(comparisonPoint.getX()) <= comparisonPoint.getY();
    }
}
