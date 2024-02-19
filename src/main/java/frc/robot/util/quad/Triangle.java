package frc.robot.util.quad; 

public class Triangle {
    public OrderedPair vertexOne, vertexTwo, vertexThree;
    public Line lineOne, lineTwo, lineThree;

    public Triangle(OrderedPair vertexOne, OrderedPair vertexTwo, OrderedPair vertexThree) {
        this.vertexOne = vertexOne;
        // System.out.println("vertexOne " + vertexOne.x + ", " + vertexOne.getY());
        this.vertexTwo = vertexTwo;
        // System.out.println("vertexTwo " + vertexTwo.x + ", " + vertexTwo.getY());
        this.vertexThree = vertexThree;
        // System.out.println("vertexThree " + vertexThree.x + ", " + vertexThree.getY());
        this.lineOne = new Line(vertexOne, vertexTwo);
        // System.out.println("Line one " + lineOne.pointA.getX() + ", " + lineOne.pointA.getY() + ", " + lineOne.pointB.getX() + ", " + lineOne.pointB.getY());
        this.lineTwo = new Line(vertexTwo, vertexThree);
        // System.out.println("Line two " + lineTwo.pointA.getX() + ", " + lineTwo.pointA.getY() + ", " + lineTwo.pointB.getX() + ", " + lineTwo.pointB.getY());
        this.lineThree = new Line(vertexThree, vertexOne);
        // System.out.println("Line three " + lineThree.pointA.getX() + ", " + lineThree.pointA.getY() + ", " + lineTwo.pointB.getX() + ", " + lineThree.pointB.getY());
        
    }

    public OrderedPair getVertexOne() {
        return vertexOne;
    }

    public OrderedPair getVertexTwo() {
        return vertexTwo;
    }

    public OrderedPair getVertexThree() {
        return vertexThree;
    }

    // Edge case when on triangle
    public boolean areInteriorPointsAboveLineOne() {
        return lineOne.isPointAboveLine(vertexThree);
    }

    public boolean areInteriorPointsAboveLineTwo() {
        return lineTwo.isPointAboveLine(vertexOne);
    }

    public boolean areInteriorPointsAboveLineThree() {
        return lineThree.isPointAboveLine(vertexTwo);
    }

    public boolean isPointInterior(OrderedPair comparisonPair) {
        if (areInteriorPointsAboveLineOne() != lineOne.isPointAboveLine(comparisonPair)) return false;
        // System.out.println(areInteriorPointsAboveLineOne() + " for line one " + lineOne.isPointAboveLine(comparisonPair));
        if (areInteriorPointsAboveLineTwo() != lineTwo.isPointAboveLine(comparisonPair)) return false;
        // System.out.println(areInteriorPointsAboveLineTwo() + " for line two " + lineTwo.isPointAboveLine(comparisonPair));
        if (areInteriorPointsAboveLineThree() != lineThree.isPointAboveLine(comparisonPair)) return false;
        // System.out.println(areInteriorPointsAboveLineThree() + " for line three " + lineThree.isPointAboveLine(comparisonPair));
        return true;
    }

}
