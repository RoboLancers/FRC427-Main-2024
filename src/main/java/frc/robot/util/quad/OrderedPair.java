package frc.robot.util.quad;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

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

    public static OrderedPair fromPose2d(Pose2d pose) {
        return new OrderedPair(pose.getX(), pose.getY()); 
    }

    public Pose2d toPose2d() {
        return new Pose2d(this.x, this.y, new Rotation2d()); 
    }

}