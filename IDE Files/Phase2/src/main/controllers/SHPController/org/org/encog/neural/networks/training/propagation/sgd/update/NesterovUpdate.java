/*
 * Encog(tm) Core v3.4 - Java Version
 * http://www.heatonresearch.com/encog/
 * https://github.com/encog/encog-java-core
 
 * Copyright 2008-2016 Heaton Research, Inc.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *   
 * For more information on Heaton Research copyrights, licenses 
 * and trademarks visit:
 * http://www.heatonresearch.com/copyright
 */
package org.encog.neural.networks.training.propagation.sgd.update;

import org.encog.neural.networks.training.propagation.sgd.StochasticGradientDescent;

/**
 * Created by jeffh on 7/15/2016.
 */
public class NesterovUpdate implements UpdateRule {

    private StochasticGradientDescent training;
    private double[] lastDelta;

    @Override
    public void init(StochasticGradientDescent theTraining) {
        this.training = theTraining;
        this.lastDelta = new double[theTraining.getFlat().getWeights().length];
    }

    @Override
    public void update(double[] gradients, double[] weights) {
        for(int i=0;i<weights.length;i++) {
            double prevNesterov = this.lastDelta[i];
            this.lastDelta[i] = (this.training.getMomentum() * prevNesterov)
                    + (gradients[i] * this.training.getLearningRate());
            final double delta = (this.training.getMomentum() * prevNesterov) - ((1+this.training.getMomentum())*this.lastDelta[i]);
            weights[i] += delta;
        }
    }
}
