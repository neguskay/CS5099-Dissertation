package main.java.kak3.shp.framework;

import java.util.Arrays;

public class ControlParameters {
    private final double[] array;

    /**
     * Represents a control space point, using a double array.
     * @param array the array.
     */
    public ControlParameters(double[] array) {
        this.array = array;
    }

    @Override
    public int hashCode() {
        return Arrays.hashCode(array);
    }

    @Override
    public boolean equals(Object other) {
        return other instanceof ControlParameters && Arrays.equals(array, ((ControlParameters) other).array);
    }

    public double[] getArray() {
        return array;
    }

    @Override
    public String toString() {
        return Arrays.toString(array);
    }

    /**
     * Translates this vector by another control space vector --- i.e. treats this is a relative co-ordinate, and
     * puts it in absolute space.
     * @param by the vector to translate relative to.
     * @return the new control parameters.
     */
    public ControlParameters translate(ControlParameters by) {
        double[] translation = by.getArray().clone();
        for (int i = 0; i < array.length; i++) {
            translation[i] += array[i];
        }

        return new ControlParameters(translation);
    }
}
