package main.java.kak3.shp.framework;

public class PositionAndHeading {
    public final Position position;
    public final double heading;

    /**
     * Represents a 2D vector co-ordinate, in addition to a heading in radians.
     * @param position the position vector.
     * @param heading the heading.
     */
    public PositionAndHeading(Position position, double heading) {
        this.position = position;
        this.heading = heading % (2 * Math.PI);
    }

    @Override
    public String toString() {
        return position + " @ " + heading;
    }
}
