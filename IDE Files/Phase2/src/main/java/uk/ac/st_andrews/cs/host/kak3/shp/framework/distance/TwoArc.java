package uk.ac.st_andrews.cs.host.kak3.SHP.framework.distance;

import uk.ac.st_andrews.cs.host.kak3.SHP.framework.Arc;
import uk.ac.st_andrews.cs.host.kak3.SHP.framework.Position;
import uk.ac.st_andrews.cs.host.kak3.SHP.framework.PositionAndHeading;

public class TwoArc implements DistanceCalculator {
    @Override
    public double getDistBetween(PositionAndHeading origin, Position intermediate, Position goal) {
        Arc first = new Arc(origin.position, intermediate, origin.heading);
        Arc second = new Arc(intermediate, goal, first.getResultantHeading());
        return first.getDistance() + second.getDistance();
    }
}