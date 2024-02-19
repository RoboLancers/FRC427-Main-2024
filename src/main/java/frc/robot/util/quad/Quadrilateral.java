package frc.robot.util.quad; 

public class Quadrilateral {
    OrderedPair[] orderedPairs;
    Triangle[] triangleArray;

    public Quadrilateral(OrderedPair vertexA, OrderedPair vertexB, OrderedPair vertexC, OrderedPair vertexD) {
        orderedPairs = new OrderedPair[4];
        orderedPairs[0] = vertexA;
        orderedPairs[1] = vertexB;
        orderedPairs[2] = vertexC;
        orderedPairs[3] = vertexD;

        triangleArray = new Triangle[4];
        for (int i = 0; i < 4; i++) {
            OrderedPair vertexOne = null;
            OrderedPair vertexTwo = null;
            OrderedPair vertexThree = null;
            for (int j = 0; j < 4; j++) {
                if ((i != j) && (vertexOne == null)) {
                    vertexOne = orderedPairs[j];
                } else if ((i != j) && (vertexTwo == null)) {
                    vertexTwo = orderedPairs[j];
                } else if ((i != j) && (vertexThree == null)) {
                    vertexThree = orderedPairs[j];
                }
            }
            triangleArray[i] = new Triangle(vertexOne, vertexTwo, vertexThree);
        }
    }

    public boolean isPointInterior(OrderedPair comparisonPoint) {
        for (int i = 0; i < 4; i++) {
            if (triangleArray[i].isPointInterior(comparisonPoint)) return true;
        }
        return false;
    }

}
