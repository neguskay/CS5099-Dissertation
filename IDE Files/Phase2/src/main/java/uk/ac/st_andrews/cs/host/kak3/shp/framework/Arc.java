package uk.ac.st_andrews.cs.host.kak3.SHP.framework;

import uk.ac.st_andrews.cs.host.kak3.SHP.framework.distance.Euclidean;

public class Arc {
    private final Position origin;
    private final Position destination;
    private final double initialHeading;
    private final Position headingVector;

    private double resultantHeading;
    private double distance;
    private double radius;

    /**
     * Represents an arc at the given initialHeading, between the origin and destination points.
     * @param origin the origin of the arc.
     * @param destination the destination of the arc.
     * @param initialHeading the initial heading at the start of the arc.
     */
    public Arc(Position origin, Position destination, double initialHeading) {
        this.origin = origin;
        this.destination = destination;
        this.initialHeading = initialHeading;
        this.headingVector = new Position(Math.cos(initialHeading), Math.sin(initialHeading));
        calculateResult();
    }

    /**
     * Called at instantiation to calculate the length of the arc and the initialHeading result.
     */
    private void calculateResult() {
        double dotProduct = headingVector.x * destination.x + headingVector.y * destination.y;
        double headingLength = Euclidean.euclideanDistance(headingVector.toArray(), new Position(0, 0).toArray());
        double destination_length = Euclidean.euclideanDistance(destination.toArray(), new Position(0, 0).toArray());
        
        
        double destinationAngle = Math.atan2(destination.y, destination.x);
        if (destinationAngle < 0) {
            destinationAngle = 2 * Math.PI + destinationAngle;
        }

        if (initialHeading > destinationAngle) {
            this.resultantHeading = initialHeading - theta;
        }
        else {
            this.resultantHeading = initialHeading + theta;
        }
    }

    public double getResultantHeading() {
        return resultantHeading;
    }

    public double getRadius() {
        return radius;
    }

    public double getDistance() {
        return distance;
    }
}
