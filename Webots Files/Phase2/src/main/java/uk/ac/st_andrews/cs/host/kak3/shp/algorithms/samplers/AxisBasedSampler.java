package uk.ac.st_andrews.cs.host.kak3.SHP.algorithms.samplers;

import uk.ac.st_andrews.cs.host.kak3.SHP.framework.ControlParameters;

import java.util.Arrays;
import java.util.Collection;
import java.util.LinkedList;
import java.util.List;

public class AxisBasedSampler implements SampleTechnique {
    private Double minParamValue = null;
    private Double maxParamValue = null;
    private double stepSize;

    /**
     * SampleTechnique which returns parameters *stepSize* along each axis, positive and negative, respecting
     * maximum and minimum parameter values.
     * @param initialStepSize initial step size to step along each axis.
     * @param minParamValue minimum allowed parameter value to set.
     * @param maxParamValue maximum allowed parameter value to set.
     */
    public AxisBasedSampler(double initialStepSize, Double minParamValue, Double maxParamValue) {
        this.stepSize = initialStepSize;
        this.minParamValue = minParamValue;
        this.maxParamValue = maxParamValue;
    }

    @Override
    public Collection<ControlParameters> getSamplesFrom(ControlParameters current) {
        List<ControlParameters> paramsToTest = new LinkedList<>();
        double[] array = current.getArray();

        for (int i = 0; i < array.length; i++) {
            double[] valsToTestPositive = Arrays.copyOf(array, array.length);
            valsToTestPositive[i] = valsToTestPositive[i] + stepSize;
            if (maxParamValue != null) {
                valsToTestPositive[i] = Double.min(valsToTestPositive[i], maxParamValue);
            }
            paramsToTest.add(new ControlParameters(valsToTestPositive));

            double[] valsToTestNegative = Arrays.copyOf(array, array.length);
            valsToTestNegative[i] = valsToTestNegative[i] - stepSize;
            if (minParamValue != null) {
                valsToTestNegative[i] = Double.max(valsToTestNegative[i], minParamValue);
            }

            paramsToTest.add(new ControlParameters(valsToTestNegative));
        }
        return paramsToTest;
    }

    @Override
    public void setStepSize(double stepSize) {
        this.stepSize = stepSize;
    }

    @Override
    public double getStepSize() {
        return stepSize;
    }

    @Override
    public double getMaxValue() {
        return maxParamValue;
    }

    @Override
    public double getMinValue() {
        return minParamValue;
    }
}
