package main.java.kak3.shp.algorithms.samplers;

import main.java.kak3.shp.framework.ControlParameters;


import java.util.ArrayList;
import java.util.Collection;
import java.util.Collections;
import java.util.List;
import java.util.stream.Collectors;

public class StaticSampler implements SampleTechnique {
    private final double minVal;
    private final double maxVal;
    private double granularity;

    private Collection<ControlParameters> result;

    /**
     * Sampler which returns all points within minVal and maxVal, with a defined granularity.
     * @param minVal the minimum parameter value to start at.
     * @param maxVal the maximum parameter value to go to.
     * @param granularity the difference between each sample point, on each axis.
     */
    public StaticSampler(double minVal, double maxVal, double granularity) {
        this.minVal = minVal;
        this.maxVal = maxVal;
        this.granularity = granularity;
    }

    /**
     * Recursive function which generates all permutations of control parameters below the passed index.
     * @param current the current array values to use.
     * @param index the index this call is responsible for permuting.
     * @return a collection of control parameters representing all permutations of values at or below the index in
     * current provided.
     */
    private Collection<ControlParameters> genNextSampleIndex(double[] current, int index) {
        if (index >= current.length) {
            return Collections.emptyList();
        }

        List<ControlParameters> params = new ArrayList<>();

        for (double num = minVal; num <= maxVal; num += granularity) {
            double[] copy = current.clone();
            copy[index] = num;
            params.add(new ControlParameters(copy));
            params.addAll(genNextSampleIndex(copy, index + 1));
        }

        return params;
    }

    @Override
    public Collection<ControlParameters> getSamplesFrom(ControlParameters current) {
        if (result == null) {
            List<ControlParameters> params = new ArrayList<>();
            double[] val = new double[current.getArray().length];
            for (int i = 0; i < val.length; i++) {
                val[i] = minVal;
            }

            double[] copy = val.clone();
            params.add(new ControlParameters(copy));
            params.addAll(genNextSampleIndex(val, 0));

            result = params;
        }

        return result.stream().map(parameters -> new ControlParameters(parameters.getArray().clone())).collect(Collectors.toCollection(ArrayList::new));
    }



    @Override
    public void setStepSize(double stepSize) {
        this.granularity = stepSize;
        this.result = null;
    }

    @Override
    public double getStepSize() {
        return granularity;
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
