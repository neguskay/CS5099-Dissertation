package main.java.kak3.shp.application.neuralnet;

import main.java.kak3.shp.framework.ControlParameters;
import org.encog.engine.network.activation.ActivationSigmoid;
import org.encog.ml.data.MLData;
import org.encog.ml.data.MLDataPair;
import org.encog.ml.data.basic.BasicMLDataSet;
import org.encog.neural.networks.BasicNetwork;
import org.encog.neural.networks.layers.BasicLayer;


/**
 * Xor example from Encog examples directory, modified to use the framework for search.
 */
public class Xor extends NeuralProblem {
    /**
     * The input necessary for XOR.
     */
    private static final double[][] XOR_INPUT = { { 0.0, 0.0 }, { 1.0, 0.0 },
            { 0.0, 1.0 }, { 1.0, 1.0 } };

    /**
     * The ideal data necessary for XOR.
     */
    private static final double[][] XOR_IDEAL = { { 0.0 }, { 1.0 }, { 1.0 }, { 0.0 } };

    public Xor() {
        // create a neural network, without using a factory
        network = new BasicNetwork();
        network.addLayer(new BasicLayer(null,true,2));
        network.addLayer(new BasicLayer(new ActivationSigmoid(),true,3));
        network.addLayer(new BasicLayer(new ActivationSigmoid(),false,1));
        network.getStructure().finalizeStructure();
        network.reset();

        // create training data
        trainingSet = new BasicMLDataSet(XOR_INPUT, XOR_IDEAL);
    }

    @Override
    ControlParameters getInitialState() {
        return null;
    }

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
