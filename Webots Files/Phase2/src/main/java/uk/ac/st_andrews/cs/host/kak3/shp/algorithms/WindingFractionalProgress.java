package uk.ac.st_andrews.cs.host.kak3.SHP.algorithms;

import uk.ac.st_andrews.cs.host.kak3.SHP.framework.ControlParameters;
import uk.ac.st_andrews.cs.host.kak3.SHP.framework.Framework;
import uk.ac.st_andrews.cs.host.kak3.SHP.algorithms.samplers.SampleTechnique;

public class WindingFractionalProgress implements SearchAlgorithm {
    private final SampleTechnique sampler;

    public WindingFractionalProgress(SampleTechnique sampler) {
        this.sampler = sampler;
    }

    private double calculateDistance(ControlParameters sample, ControlParameters current) {
        double[] samplePoints = sample.getArray();
        double sum = 0;
        for (int i = 0; i < samplePoints.length; i++) {
            sum += Math.pow(samplePoints[i] - current.getArray()[i], 2);
        }

        return Math.sqrt(sum);
    }

    @Override
    public void iteration(Framework framework) {
        Double bestRatio = null;
        ControlParameters bestSample = null;

        for (ControlParameters sample : sampler.getSamplesFrom(framework.getCurrent())) {
            double future;

            /*if (framework.getWind() <= 0) {
                future = framework.positiveTwoArcDistance(sample);
            }
            else {
                future = framework.negativeTwoArcDistance(sample);
            }

            double past = calculateDistance(sample, framework.getCurrent());
            double ratio = past / (past + future);

            if (bestRatio == null || ratio > bestRatio) {
                bestRatio = ratio;
                bestSample = sample;
            }*/
        }

        framework.move(bestSample);
    }

    @Override
    public void terminate(Framework framework) {
        framework.finished();
    }
}
