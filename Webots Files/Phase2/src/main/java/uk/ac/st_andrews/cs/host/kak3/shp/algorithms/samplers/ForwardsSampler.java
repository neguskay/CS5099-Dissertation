package uk.ac.st_andrews.cs.host.kak3.SHP.algorithms.samplers;

import uk.ac.st_andrews.cs.host.kak3.SHP.framework.ControlParameters;

import java.util.Collection;
import java.util.HashSet;

public class ForwardsSampler implements SampleTechnique {
    private final double minVal;
    private final double maxVal;
    private double stepSize;

    /**
     * Sampler which returns points "forwards" from the current control space vector. These will be points on the
     * surface of the forwards hyper-hemisphere.
     * @param minVal the minimum allowed parameter value.
     * @param maxVal the maximum allowed parameter value.
     * @param stepSize radius of the hyper-hemisphere.
     */
    public ForwardsSampler(double minVal, double maxVal, double stepSize) {
        this.minVal = minVal;
        this.maxVal = maxVal;
        this.stepSize = stepSize;
    }

    private double calcMinVal(double[] vector, double[] vectorComponents) {
        double sum = 0;
        for (int i = 0; i < vector.length - 1; i++) {
            sum += vector[i]*vectorComponents[i];
        }
        return -sum / vector[vector.length - 1];
    }

    private Collection<ControlParameters> generateProbes(ControlParameters current, Collection<ControlParameters> vectors) {
        Collection<ControlParameters> probes = new HashSet<>();
        for (ControlParameters vector : vectors) {
            double min = calcMinVal(current.getArray(), vector.getArray());
            for (double a_last = min; a_last < min + 5; a_last++) {
                double nextProbe[] = new double[current.getArray().length];
                vector.getArray()[vector.getArray().length - 1] = a_last;
                double mag = 0;
                for (int i = 0; i < current.getArray().length; i++) {
                    mag += Math.pow(vector.getArray()[i], 2);
                }
                double lambda = stepSize / Math.sqrt(mag);
                for (int i = 0; i < current.getArray().length; i++) {
                    nextProbe[i] = lambda*vector.getArray()[i];
                }
                probes.add(new ControlParameters(nextProbe));
            }
        }
        return probes;
    }

    @Override
    public Collection<ControlParameters> getSamplesFrom(ControlParameters current) {
        SampleTechnique sampler = new StaticSampler(-10, 10, 3);
        Collection<ControlParameters> vectors = sampler.getSamplesFrom(current);
        return generateProbes(current, vectors);
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
