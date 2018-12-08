package uk.ac.st_andrews.cs.host.kak3.SHP.algorithms.samplers;

import uk.ac.st_andrews.cs.host.kak3.SHP.framework.ControlParameters;

import java.util.Collection;

public interface SampleTechnique {

    /**
     * Should return all of the samples in control space around the given control space point, depending on the
     * specific mechanism employed in the subclass.
     * @param current the current control space parameters, to sample from.
     * @return the samples around the given point.
     */
    Collection<ControlParameters> getSamplesFrom(ControlParameters current);
    void setStepSize(double stepSize);
    double getStepSize();
    double getMaxValue();
    double getMinValue();
}
