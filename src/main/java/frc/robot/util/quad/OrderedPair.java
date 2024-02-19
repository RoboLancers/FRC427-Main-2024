package frc.robot.util.quad; 

public class OrderedPair {

    private double x, y;

    public OrderedPair(double x, double y) {
        this.x = x;
        this.y = y;
    }

    public double getX() {
        return this.x;
    }

    public double getY() {
        return this.y;
    }

    public double getSlopeBetweenPoints(OrderedPair secondPair) {
        if (secondPair.getX() == this.x) {
            return Double.POSITIVE_INFINITY;
        }
        return ((secondPair.getY() - this.y) / (secondPair.getX() - this.x));
    }

    public double getInterceptBetweenPoints(OrderedPair secondPair) {
        double slope = getSlopeBetweenPoints(secondPair);
        if (slope == Double.POSITIVE_INFINITY) {
            return Double.POSITIVE_INFINITY;
        }
        return (this.y - this.x * slope);
    }
}