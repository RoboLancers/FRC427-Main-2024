package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.util.quad.OrderedPair;

public class GeometryUtils {

    public static boolean isPoseInRectangle(Pose2d pose, OrderedPair topLeft, OrderedPair bottomRight) {
        return topLeft.getX() <= pose.getX()
               && bottomRight.getX() >= pose.getX() 
               && topLeft.getY() >= pose.getY() 
               && bottomRight.getY() <= pose.getY();  
    }
    
}
