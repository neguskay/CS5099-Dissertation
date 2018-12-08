package main.java.kak3.shp.framework.distance;


import main.java.kak3.shp.framework.Position;
import main.java.kak3.shp.framework.PositionAndHeading;

/**
 * Represents a distance measure from an origin position and heading, via an intermediate sample point, to the goal.
 */
public interface DistanceCalculator {
    double getDistanceBetween(PositionAndHeading origin, Position intermediate, Position goal);
    double getDistanceBetween(Position origin, Position destination);
}