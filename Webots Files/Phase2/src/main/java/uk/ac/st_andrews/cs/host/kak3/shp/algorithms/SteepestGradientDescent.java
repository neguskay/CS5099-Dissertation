package uk.ac.st_andrews.cs.host.kak3.SHP.algorithms;

import uk.ac.st_andrews.cs.host.kak3.SHP.framework.ControlParameters;
import uk.ac.st_andrews.cs.host.kak3.SHP.framework.Framework;
import uk.ac.st_andrews.cs.host.kak3.SHP.algorithms.samplers.SampleTechnique;
import uk.ac.st_andrews.cs.host.kak3.SHP.framework.distance.Euclidean;

import java.util.Collection;
import java.util.HashMap;
import java.util.Map;

public class SteepestGradientDescent implements SearchAlgorithm {
    private final SampleTechnique sampler;
    private final double minGradient;
    private final boolean onlyMoveToSamples;
    private final double learningRate;

    /**
     * Instantiate a SteepestGradientDescent algorithm. It will generate initial random control parameters.
     * @param sampler the technique to use to generate samples from the current point.
     * @param minGradient the minimum gradient of move that will be executed. If there is no move with a greater
     *                    gradient, the algorithm will terminate.
     * @param learningRate the learning rate to apply. -1 indicates to keep the initial step size at all times.
     * @param onlyMoveToSamples if true, the algorithm will doMove directly to the best sample, otherwise it will
     *                           make a doMove elsewhere based on a ratio between the gradient of sample points.
     */
    public SteepestGradientDescent(SampleTechnique sampler, double minGradient, double learningRate, boolean onlyMoveToSamples) {
        this.sampler = sampler;
        this.minGradient = minGradient;
        this.learningRate = learningRate;
        this.onlyMoveToSamples = onlyMoveToSamples;
    }

    /**
     * Make a move directly to the best sample.
     * @param framework the framework instance.
     * @param bestSample the best sample to move to.
     */
    private void makeDirectMove(Framework framework, ControlParameters bestSample) {
        framework.move(bestSample);
    }

    /**
     * Make a move to some intermediate point based on all of the costs, rather than directly to a sample.
     * @param framework the framework instance.
     * @param largestReduction the largest reduction in cost we saw.
     * @param gradientMap a map from neighbours to their gradients.
     */
    private void makeRatioMove(Framework framework, double largestReduction, Map<ControlParameters, Double> gradientMap) {
        double totalRatioValues = 0;

        double[] newVals = new double[framework.getCurrent().getArray().length];
        for (Map.Entry<ControlParameters, Double> sample : gradientMap.entrySet()) {
            double[] sampleVals = sample.getKey().getArray();
            double ratio = sample.getValue() / largestReduction;

            if (ratio > 0) {
                totalRatioValues += ratio;
                for (int i = 0; i < sampleVals.length; i++) {
                    newVals[i] += sampleVals[i] * ratio;
                }
            }
        }

        for (int i = 0; i < newVals.length; i++) {
            newVals[i] /= totalRatioValues;
        }
        framework.move(new ControlParameters(newVals));
    }

    @Override
    public void iteration(Framework framework) {
        double[] vals = framework.getCurrent().getArray();

        double currentCost = framework.getCurrentCost();
        Collection<ControlParameters> samples = sampler.getSamplesFrom(new ControlParameters(vals));

        // Map each sample point to its cost reduction, and find the one with the largest reduction.
        ControlParameters bestSample = null;
        Double steepestGradient = null;
        Map<ControlParameters, Double> gradientMap = new HashMap<>();
        for (ControlParameters sample : samples) {
            double cost = framework.getCost(sample);
            double reduction = currentCost - cost;
            double distance = Euclidean.euclideanDistance(sample, new ControlParameters(vals));

            // Exclude infinite reductions, or the same point we're at, where we'll get a divide by 0.
            if (distance == 0 || Double.isInfinite(reduction)) {
                continue;
            }

            double gradient = reduction / distance;
            if (steepestGradient == null || gradient > steepestGradient) {
                steepestGradient = gradient;
                bestSample = sample;
            }

            gradientMap.put(sample, reduction);
        }

        assert steepestGradient != null;
        assert bestSample != null;
/************************************************************/
/************************************************************/
/* change ZAH  :   set set size to (.1 )
/************************************************************/
 
        if (learningRate != -1) {
            sampler.setStepSize(learningRate * steepestGradient);
        }
/************************************************************/
/* change ZAH  :  removing the if statement
/************************************************************/
        
        // If no move reduces the error, we're in a minimum - terminate.
        if (steepestGradient < minGradient) {
            System.out.println("Terminating due to min gradient!");
            terminate(framework);
            return;
        }
/************************************************************/
/************************************************************/

        // Depending on our arguments, either move to the best sample we've got, or some intermediate point.
        if (onlyMoveToSamples) {
            makeDirectMove(framework, bestSample);
        }
        else {
            makeRatioMove(framework, steepestGradient, gradientMap);
        }
    }

    @Override
    public void terminate(Framework framework) {
        framework.finished();
    }
}
