package main.java.kak3.shp.framework.distance;


import main.java.kak3.shp.framework.ControlParameters;
import main.java.kak3.shp.framework.Position;
import main.java.kak3.shp.framework.PositionAndHeading;

public class Euclidean implements DistanceCalculator {
    @Override
    public double getDistanceBetween(PositionAndHeading origin, Position intermediate, Position goal) {
        return euclideanDistance(intermediate.toArray(), goal.toArray());
    }

    @Override
    public double getDistanceBetween(Position origin, Position destination) {
	System.out.println("Current: "+ origin);
      System.out.println("Obstacle: "+ destination);
	
        return euclideanDistance(origin.toArray(), destination.toArray());
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
