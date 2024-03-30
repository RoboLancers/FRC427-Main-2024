package frc.robot.util;

import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants;
import frc.robot.util.quad.Line;
import frc.robot.util.quad.OrderedPair;
import frc.robot.util.quad.Triangle;
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

    public static boolean isInRedTriangle(Pose2d robotPose) {
        return new Triangle(new OrderedPair(0.0,0.0), new OrderedPair(0.0,0.0), new OrderedPair(0.0,0.0)).isPointInterior(new OrderedPair(robotPose.getX(), robotPose.getY()));
    }

    public static boolean isInBlueTriangle(Pose2d robotPose) {
        return new Triangle(new OrderedPair(0.0,0.0), new OrderedPair(0.0,0.0), new OrderedPair(0.0,0.0)).isPointInterior(new OrderedPair(robotPose.getX(), robotPose.getY()));
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

    public static double getAngleToStage(Pose2d robotPose) {
        Alliance alliance = getTargetPose();

        if (alliance == Alliance.Blue) {
            if (isPoseInRectangle(new OrderedPair(robotPose.getX(), robotPose.getY()), 
                Constants.AutoHang.blueBottomLeft1,
                Constants.AutoHang.blueBottomRight1,
                Constants.AutoHang.blueTopRight1,
                Constants.AutoHang.blueTopLeft1
            )) {
                return 120;
            }
            else if (isPoseInRectangle(new OrderedPair(robotPose.getX(), robotPose.getY()), 
                Constants.AutoHang.blueBottomLeft2,
                Constants.AutoHang.blueBottomRight2,
                Constants.AutoHang.blueTopRight2,
                Constants.AutoHang.blueTopLeft2
            )) {
                return 0;
            }
            else if (isPoseInRectangle(new OrderedPair(robotPose.getX(), robotPose.getY()), 
                Constants.AutoHang.blueBottomLeft3,
                Constants.AutoHang.blueBottomRight3,
                Constants.AutoHang.blueTopRight3,
                Constants.AutoHang.blueTopLeft3
            )) {
                return -120;
            }
        }
        if (alliance == Alliance.Red) {
            if (isPoseInRectangle(new OrderedPair(robotPose.getX(), robotPose.getY()), 
                Constants.AutoHang.redBottomLeft1,
                Constants.AutoHang.redBottomRight1,
                Constants.AutoHang.redTopRight1,
                Constants.AutoHang.redTopLeft1
            )) {
                return 60;
            }
            else if (isPoseInRectangle(new OrderedPair(robotPose.getX(), robotPose.getY()), 
                Constants.AutoHang.redBottomLeft2,
                Constants.AutoHang.redBottomRight2,
                Constants.AutoHang.redTopRight2,
                Constants.AutoHang.redTopLeft2
            )) {
                return 180;
            }
            else if (isPoseInRectangle(new OrderedPair(robotPose.getX(), robotPose.getY()), 
                Constants.AutoHang.redBottomLeft3,
                Constants.AutoHang.redBottomRight3,
                Constants.AutoHang.redTopRight3,
                Constants.AutoHang.redTopLeft3
            )) {
                return -60;
            }
        }

        return 0; 
    }

    public static Alliance getTargetPose() {
        Optional<Alliance> optAlliance = DriverStation.getAlliance();

        if (optAlliance.isEmpty()) return null;

        Alliance alliance = optAlliance.get(); 
        return alliance; 
    }

    public static boolean isPoseInRectangle(OrderedPair position, OrderedPair topLeft, OrderedPair bottomRight, OrderedPair topRight, OrderedPair bottomLeft) {
        Quadrilateral quad = new Quadrilateral(topLeft, bottomRight, topRight, bottomLeft); 

        return quad.isPointInterior(position); 
    }
    
}