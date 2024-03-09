package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.util.quad.Line;
import frc.robot.util.quad.OrderedPair;
import frc.robot.util.quad.Quadrilateral;
import frc.robot.util.quad.Triangle;

public class GeometryUtils {

    public static boolean isPoseInRectangle(Pose2d pose, OrderedPair topLeft, OrderedPair bottomRight) {
        return topLeft.getX() <= pose.getX()
               && bottomRight.getX() >= pose.getX() 
               && topLeft.getY() >= pose.getY() 
               && bottomRight.getY() <= pose.getY();  
    }

    public static boolean isPoseInQuadrilateral(Pose2d pose, OrderedPair p1, OrderedPair p2, OrderedPair p3, OrderedPair p4) {
        Quadrilateral quad = new Quadrilateral(p1, p2, p3, p4); 
        return quad.isPointInterior(OrderedPair.fromPose2d(pose)); 
    }

    public static OrderedPair getBisector(OrderedPair basePair, OrderedPair oppositePair1, OrderedPair oppositePair2) {
        Line bisectedLine = new Line(oppositePair1, oppositePair2);
        double bisectedLineLength = bisectedLine.getLength();
        double adjacentLine1Length = new Line(oppositePair1, basePair).getLength();
        double adjacentLine2Length = new Line(oppositePair2, basePair).getLength();
        double distantceFromOppositePair1 = adjacentLine1Length * bisectedLineLength / (adjacentLine1Length + adjacentLine2Length);
        double rightTriangleAngle = Math.atan2(bisectedLine.getDeltaY(), bisectedLine.getDeltaX());

        double oppositeDistance = bisectedLineLength - distantceFromOppositePair1; 

        double yCoordinate = - oppositeDistance * Math.sin(rightTriangleAngle) + oppositePair1.getY();
        double xCoordinate = - oppositeDistance * Math.cos(rightTriangleAngle) + oppositePair1.getX();
        return new OrderedPair(xCoordinate, yCoordinate);
    }

    public static boolean isInTriangle(OrderedPair pointA, OrderedPair pointB, OrderedPair pointC, OrderedPair comparisonPoint) {
    Triangle triangle = new Triangle(pointA, pointB, pointC);
        return triangle.isPointInterior(comparisonPoint);
    }
    public static boolean isInRedStage(OrderedPair robotPose) {
        return isInTriangle(new OrderedPair(13.42, 4.09), new OrderedPair(10.88, 5.62), new OrderedPair(10.88, 2.61), robotPose);
    }
    public static boolean isInBlueStage(OrderedPair robotPose) {
       
        return isInTriangle(new OrderedPair(3.11, 4.09), new OrderedPair(5.73, 5.62), new OrderedPair(5.73, 2.61), robotPose);
    }
    
}
