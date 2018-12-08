package uk.ac.st_andrews.cs.host.kak3.SHP.framework.distance;

import uk.ac.st_andrews.cs.host.kak3.SHP.framework.ControlParameters;
import uk.ac.st_andrews.cs.host.kak3.SHP.framework.Position;
import uk.ac.st_andrews.cs.host.kak3.SHP.framework.PositionAndHeading;

public class Euclidean implements DistanceCalculator {
    @Override
    public double getDistBetween(PositionAndHeading origin, Position intermediate, Position goal) {
        return euclideanDistance(intermediate.toArray(), goal.toArray());
    }

    public static double euclideanDistance(double[] point1, double[] point2) {
        double sum = 0;

        for (int i = 0; i < point1.length; i++) {
            sum += Math.pow(point1[i] - point2[i], 2);
        }

        return Math.sqrt(sum);
    }

    public static double euclideanDistance(ControlParameters point1, ControlParameters point2) {
        return euclideanDistance(point1.getArray(), point2.getArray());
    }
}
