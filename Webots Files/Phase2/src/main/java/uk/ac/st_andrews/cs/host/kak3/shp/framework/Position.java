package uk.ac.st_andrews.cs.host.kak3.SHP.framework;

public class Position {
    public final double x;
    public final double y;

    /**
     * Represents a 2D double vector
     * @param x the x co-ordinate.
     * @param y the y co-ordinate.
     */
    public Position(double x, double y) {
        this.x = x;
        this.y = y;
    }

    @Override
    public String toString() {
        return "(" + x + ", " + y + ")";
    }

    public double[] toArray() {
        return new double[]{x, y};
    }
}
