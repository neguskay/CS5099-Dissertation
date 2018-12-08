package uk.ac.st_andrews.cs.host.kak3.SHP.algorithms.samplers;

import uk.ac.st_andrews.cs.host.kak3.SHP.framework.ControlParameters;

import java.util.Collection;
import java.util.LinkedList;
import java.util.Random;

public class RandomSampler implements SampleTechnique {
    private final Random random = new Random();

    private final int numSamples;
    private double stepSize;
    private final Double minVal;
    private final Double maxVal;

    /**
     * Sampler which returns random points within stepSize along each axis, respecting maximum and minimum parameter
     * values.
     * @param numSamples number of samples to return.
     * @param stepSize maximum distance along each axis to sample.
     * @param minVal minimum allowed parameter value to set.
     * @param maxVal maximum allowed parameter value to set.
     */
    public RandomSampler(int numSamples, double stepSize, Double minVal, Double maxVal) {
        this.numSamples = numSamples;
        this.stepSize = stepSize;
        this.minVal = minVal;
        this.maxVal = maxVal;
    }

    @Override
    public Collection<ControlParameters> getSamplesFrom(ControlParameters current) {
        Collection<ControlParameters> params = new LinkedList<>();
        double[] currentParams = current.getArray();

        for (int sample = 0; sample < numSamples; sample += 2) {
            double[] sampleParamsPositive = new double[currentParams.length];
            double[] sampleParamsNegative = new double[currentParams.length];
            for (int i = 0; i < currentParams.length; i++) {
                double randomValue = stepSize * random.nextDouble();
                sampleParamsPositive[i] = currentParams[i] + randomValue;
                if (maxVal != null) {
                    sampleParamsPositive[i] = Double.min(sampleParamsPositive[i], maxVal);
                }
                sampleParamsNegative[i] = currentParams[i] - randomValue;
                if (minVal != null) {
                    sampleParamsNegative[i] = Double.max(sampleParamsNegative[i], minVal);
                }
            }

            params.add(new ControlParameters(sampleParamsPositive));
            params.add(new ControlParameters(sampleParamsNegative));
        }

        return params;
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
        return maxVal;
    }

    @Override
    public double getMinValue() {
        return minVal;
    }
}
