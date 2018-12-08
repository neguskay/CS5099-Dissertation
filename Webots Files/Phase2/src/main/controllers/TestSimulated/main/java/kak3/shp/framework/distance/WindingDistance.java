package main.java.kak3.shp.framework.distance;

import main.java.kak3.shp.framework.Arc;
import main.java.kak3.shp.framework.Position;
import main.java.kak3.shp.framework.PositionAndHeading;

// Attempt at strain methodology as described by Mike. Unsuccessful.
public class WindingDistance implements DistanceCalculator {

    private double getStrain(Arc arc) {
        return arc.getDistance() / arc.getRadius();
    }

    @Override
    public double getDistBetween(PositionAndHeading origin, Position intermediate, Position goal) {
        Arc first = new Arc(origin.position, intermediate, origin.heading);
        double sampleHeading = first.getResultantHeading();
        double[] headingVector = new double[2];
        headingVector[0] = Math.cos(sampleHeading);
        headingVector[1] = Math.sin(sampleHeading);
        double[] line = new double[2];
        line[0] = goal.x - intermediate.x;
        line[1] = goal.y - intermediate.y;
        double dotProduct = headingVector[0]*line[0] + headingVector[1]*line[1];
        double alpha = Math.acos(dotProduct / (
                Euclidean.euclideanDistance(headingVector, new double[]{0, 0}) *
                        Euclidean.euclideanDistance(line, new double[]{0, 0}))
        );
        double beta = 0.5837 * alpha;
        double gamma = Math.PI - 0.5 * alpha - 0.5 * beta;
        double base = Euclidean.euclideanDistance(intermediate.toArray(), goal.toArray());
        double chordAlpha = (base / Math.sin(gamma)) * (Math.sin(0.5 * beta));
        Position v = new Position(intermediate.x + chordAlpha * Math.cos(alpha * 0.5), intermediate.y + chordAlpha * Math.sin(alpha * 0.5));
        Arc second = new Arc(intermediate, v, sampleHeading);
        Arc third = new Arc(v, goal, second.getResultantHeading());

        return getStrain(first) + getStrain(second) + getStrain(third);
    }

    private double calculateStrain(PositionAndHeading origin, Position intermediate, Position goal, double initialWinding) {
        Arc first = new Arc(origin.position, intermediate, origin.heading);
        double sampleHeading = first.getResultantHeading();
        double[] headingVector = new double[2];
        headingVector[0] = Math.cos(sampleHeading);
        headingVector[1] = Math.sin(sampleHeading);
        double[] line = new double[2];
        line[0] = goal.x - intermediate.x;
        line[1] = goal.y - intermediate.y;
        double dotProduct = headingVector[0]*line[0] + headingVector[1]*line[1];
        double alphaInitial = Math.acos(dotProduct / (
                Euclidean.euclideanDistance(headingVector, new double[]{0, 0}) *
                        Euclidean.euclideanDistance(line, new double[]{0, 0}))
        );
        double alpha = 0;
        double betaInitial = 0.5837 * alpha;
        double beta = 0;
        if (initialWinding > 0) {
            alpha = 2*Math.PI - alphaInitial;
            beta = 2*Math.PI - betaInitial;
        }
        else {
            alpha = alphaInitial;
            beta = betaInitial;
        }
        double gamma = Math.PI - 0.5 * alphaInitial - 0.5 * betaInitial;
        double base = Euclidean.euclideanDistance(intermediate.toArray(), goal.toArray());
        double chord_alpha = (base / Math.sin(gamma)) * (Math.sin(0.5 * betaInitial));
        Position v = new Position(intermediate.x + chord_alpha * Math.cos(alphaInitial * 0.5), intermediate.y + chord_alpha * Math.sin(alphaInitial * 0.5));
        Arc second = new Arc(intermediate, v, sampleHeading);
        Arc third = new Arc(v, goal, second.getResultantHeading());

        return getStrain(first) + getStrain(second) + getStrain(third);
    }


}
