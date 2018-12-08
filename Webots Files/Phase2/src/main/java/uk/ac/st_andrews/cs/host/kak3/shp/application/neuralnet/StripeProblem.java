package uk.ac.st_andrews.cs.host.kak3.SHP.application.neuralnet;

import org.encog.engine.network.activation.ActivationSigmoid;
import org.encog.ml.data.MLData;
import org.encog.ml.data.MLDataPair;
import org.encog.ml.data.basic.BasicMLDataSet;
import org.encog.neural.networks.BasicNetwork;
import org.encog.neural.networks.layers.BasicLayer;
import uk.ac.st_andrews.cs.host.kak3.SHP.framework.ControlParameters;

public class StripeProblem extends NeuralProblem {
    private static final double HIGH_IDEAL = 0.8;
    private static final double LOW_IDEAL = 0.2;

    private static final double COORD_SIZE = 10.0;

    private static final double[][] STRIPE_INPUT = {
            { -1.0, -2.0 },
            {  0.0, -2.0 },
            {  1.0, -2.0 },
            {  0.0, -1.0 },
            {  0.0, 1.0},
            { -1.0, 2.0},
            {  0.0, 2.0},
            {  1.0, 2.0}

    };

    private static final double[][] STRIPE_IDEAL = {
            { LOW_IDEAL },
            { HIGH_IDEAL },
            { LOW_IDEAL },
            { HIGH_IDEAL },
            { HIGH_IDEAL },
            { LOW_IDEAL },
            { HIGH_IDEAL },
            { LOW_IDEAL }
    };

    // Training for the "bad" one
    /*private static double STRIPE_IDEAL[][] = {
            { LOW_IDEAL },
            { LOW_IDEAL },
            { LOW_IDEAL },
            { LOW_IDEAL },
            { HIGH_IDEAL },
            { HIGH_IDEAL },
            { HIGH_IDEAL },
            { HIGH_IDEAL }
    };*/

    private void createNetwork() {
        network = new BasicNetwork();
        network.addLayer(new BasicLayer(null, false, 2));
        network.addLayer(new BasicLayer(new ActivationSigmoid(), true, 2));
        network.addLayer(new BasicLayer(new ActivationSigmoid(), true, 1));
        network.getStructure().finalizeStructure();
    }

    private void setTrainingSet() {
        double[][] adjusted = STRIPE_INPUT.clone();
        for (int i = 0; i < adjusted.length; i++) {
            for (int j = 0; j < adjusted[i].length; j++) {
                adjusted[i][j] *= COORD_SIZE;
            }
        }
        trainingSet = new BasicMLDataSet(adjusted, STRIPE_IDEAL);
    }

    public StripeProblem() {
        createNetwork();
        setTrainingSet();
    }

    @Override
    ControlParameters getInitialState() {
        double[] vals = new double[] {
                1.0739627186764635,
                1.4590236544697912,
                -1.3128439570581398,
                -0.4455337025184384,
                0.704235270230297,
                0.6447946232901867,
                0.5174206990717445
        };

        /*network.reset();
        double[] vals = new double[7];
        String weights = network.dumpWeights();
        String[] weight_strs = weights.split(",");
        for (int i = 0; i < weight_strs.length; i++) {
            vals[i] = Double.parseDouble(weight_strs[i]);
        }*/

        return new ControlParameters(vals);
    }

    /*private int numIncorrect() {
        int wrong = 0;
        for (MLDataPair pair : getTrainingSet()) {
            final MLData output = getNetwork().compute(pair.getInput());
            if (Math.abs(output.getData(0) - pair.getIdeal().getData(0)) > 0.2) {
                wrong++;
            }
        }
        return wrong;
    }*/

    @Override
    public void finished() {
        for(MLDataPair pair: getTrainingSet() ) {
            final MLData output = getNetwork().compute(pair.getInput());
            System.out.println(pair.getInput().getData(0) + "," + pair.getInput().getData(1)
                    + ", actual=" + output.getData(0) + ",ideal=" + pair.getIdeal().getData(0));
        }

        shutdown();
    }
}
