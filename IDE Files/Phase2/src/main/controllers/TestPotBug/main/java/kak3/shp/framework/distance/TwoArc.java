package main.java.kak3.shp.framework.distance;

import main.java.kak3.shp.framework.Arc;
import main.java.kak3.shp.framework.Position;
import main.java.kak3.shp.framework.PositionAndHeading;


public class TwoArc implements DistanceCalculator {
    @Override
    public double getDistBetween(PositionAndHeading origin, Position intermediate, Position goal) {
        Arc first = new Arc(origin.position, intermediate, origin.heading);
        Arc second = new Arc(intermediate, goal, first.getResultantHeading());
        return first.getDistance() + second.getDistance();
    }


}