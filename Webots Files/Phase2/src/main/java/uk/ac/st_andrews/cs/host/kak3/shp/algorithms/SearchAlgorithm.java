package uk.ac.st_andrews.cs.host.kak3.SHP.algorithms;

import uk.ac.st_andrews.cs.host.kak3.SHP.framework.Framework;

public interface SearchAlgorithm {
    void iteration(Framework framework);
    void terminate(Framework framework);
}
