package uk.ac.st_andrews.cs.host.kak3.SHP.framework.distance;

import uk.ac.st_andrews.cs.host.kak3.SHP.framework.Position;
import uk.ac.st_andrews.cs.host.kak3.SHP.framework.PositionAndHeading;

/**
 * Represents a distance measure from an origin position and heading, via an intermediate sample point, to the goal.
 */
public interface DistanceCalculator {
    double getDistBetween(PositionAndHeading origin, Position intermediate, Position goal);
}