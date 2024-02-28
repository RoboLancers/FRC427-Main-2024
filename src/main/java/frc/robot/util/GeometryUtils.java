package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.util.quad.Line;
import frc.robot.util.quad.OrderedPair;

public class GeometryUtils {

    public static boolean isPoseInRectangle(Pose2d pose, OrderedPair topLeft, OrderedPair bottomRight) {
        return topLeft.getX() <= pose.getX()
               && bottomRight.getX() >= pose.getX() 
               && topLeft.getY() >= pose.getY() 
               && bottomRight.getY() <= pose.getY();  
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

    
    
}
