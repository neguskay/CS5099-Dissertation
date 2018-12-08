package uk.ac.st_andrews.cs.host.kak3.SHP.framework;

import java.io.FileWriter;
import java.io.IOException;
import java.io.PrintWriter;
import java.util.ArrayList;
import java.util.List;

/**
 * Stats gathering class, allowing frameworks/components to call it to count various things, and then call its dump()
 * method to write to stdout/file. Will only write once, to prevent overwriting data.
 */
public class StatsGatherer {
    private int numIterations = 0;
    private int numCosted = 0;
    private int numSequencesCosted = 0;
    private List<Double> costs = new ArrayList<>();
    private long startTime;

    private boolean wrote = false;

    private String statsPath;

    StatsGatherer(String path) {
        this.statsPath = path;
        startTime = System.currentTimeMillis();
    }

    public void iteration() {
        numIterations++;
    }

    void costed() {
        numCosted++;
    }

    void sequenceCosted() {
        numSequencesCosted++;
    }

    void addMoveCost(double cost) {
        costs.add(cost);
    }

    /**
     * Dumps stat information to STDOUT, and writes CSV file with cost over time.
     */
    public void dump() {
        // Only write if we haven't written before.
        if (wrote) {
            return;
        }
        wrote = true;

        System.out.println("Num iterations: " + numIterations);
        System.out.println("Num costed: " + numCosted);
        System.out.println("Num sequence costed: " + numSequencesCosted);
        System.out.println("Real time: " + (System.currentTimeMillis() - startTime) + " ms");

        try {
            PrintWriter writer = new PrintWriter(new FileWriter(statsPath));
            writer.println("iteration,cost");
            for (int i = 0; i < costs.size(); i++) {
                writer.println(i + "," + costs.get(i));
            }
            writer.close();
        } catch (IOException e) {
            System.out.println("Failed to write stats file.");
            e.printStackTrace();
        }

    }

}
