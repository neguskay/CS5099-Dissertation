package main.java.kak3.shp.framework.distance;


import main.java.kak3.shp.framework.Arc;
import main.java.kak3.shp.framework.Position;
import main.java.kak3.shp.framework.PositionAndHeading;

public class ThreeArc implements DistanceCalculator {

    public double getDistBetween(PositionAndHeading origin, Position intermediate, Position goal) {
        // Following Mike's methodology as described for CS5011: https://studres.cs.st-andrews.ac.uk/CS5011/Practicals/Robotics%20Practical%20arc%20construction%20v2.pdf

        Arc first = new Arc(origin.position, intermediate, origin.heading);
        double sampleHeading = first.getResultantHeading();
        double[] heading_vector = new double[2];
        heading_vector[0] = Math.cos(sampleHeading);
        heading_vector[1] = Math.sin(sampleHeading);
        double[] line = new double[2];
        line[0] = goal.x - intermediate.x;
        line[1] = goal.y - intermediate.y;
        double dotProduct = heading_vector[0]*line[0] + heading_vector[1]*line[1];
        double alpha = Math.acos(dotProduct / (
                Euclidean.euclideanDistance(heading_vector, new double[]{0, 0}) *
                        Euclidean.euclideanDistance(line, new double[]{0, 0}))
        );
        double beta = 0.5837 * alpha;
        double gamma = Math.PI - 0.5 * alpha - 0.5 * beta;
        double base = Euclidean.euclideanDistance(intermediate.toArray(), goal.toArray());
        double chord_alpha = (base / Math.sin(gamma)) * (Math.sin(0.5 * beta));
        
        
        Position v = new Position(intermediate.x + chord_alpha * Math.cos(alpha * 0.5), intermediate.y + chord_alpha * Math.sin(alpha * 0.5));
        Arc second = new Arc(intermediate, v, sampleHeading);
        Arc third = new Arc(v, goal, second.getResultantHeading());
        return first.getDistance() + second.getDistance() + third.getDistance();
    }




    @Override
    public double getDistanceBetween(Position origin, Position destination) {
        return 0;
    }
}
