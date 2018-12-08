package uk.ac.st_andrews.cs.host.kak3.SHP.algorithms;

import org.apache.commons.math3.linear.ArrayRealVector;
import uk.ac.st_andrews.cs.host.kak3.SHP.framework.ControlParameters;

public class WindingUtil {
    /**
     * Calculates the angle to the X axis in n dimensions.
     * @param parameters the control space vector.
     * @return the angle.
     */
    private static double calculateDirection(ControlParameters parameters) {
        ArrayRealVector vector = new ArrayRealVector(parameters.getArray(), true);
        double[] xAxis = new double[vector.getDimension()];
        xAxis[0] = 1;
        ArrayRealVector xAxisVector = new ArrayRealVector(xAxis);

        double dotProduct = vector.dotProduct(xAxisVector);
        double denominator = vector.getNorm() * xAxisVector.getNorm();

        return Math.acos(dotProduct / denominator);
    }

    /**
     * Should be called once at least two moves have been executed, to return the amount wind should be changed by.
     * @param prevVals the previous control parameter vector.
     * @param newVals the new control parameter vector.
     */
    public static double calcWindingDelta(ControlParameters prevVals, ControlParameters newVals) {
        double previousDirection = calculateDirection(prevVals);
        double newDirection = calculateDirection(newVals);

        double angle = previousDirection - newDirection;
        if (angle > Math.PI) {
            angle = -1 * (2*Math.PI - angle);
        }
        else if (angle < -Math.PI) {
            angle = 2*Math.PI + angle;
        }

        return angle;
    }
}
