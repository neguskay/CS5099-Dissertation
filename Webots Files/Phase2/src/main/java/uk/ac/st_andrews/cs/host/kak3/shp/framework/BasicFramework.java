package uk.ac.st_andrews.cs.host.kak3.SHP.framework;

import net.sourceforge.argparse4j.ArgumentParsers;
import net.sourceforge.argparse4j.impl.Arguments;
import net.sourceforge.argparse4j.inf.ArgumentParser;
import net.sourceforge.argparse4j.inf.ArgumentParserException;
import net.sourceforge.argparse4j.inf.Namespace;
import uk.ac.st_andrews.cs.host.kak3.SHP.algorithms.*;
import uk.ac.st_andrews.cs.host.kak3.SHP.algorithms.samplers.*;
import uk.ac.st_andrews.cs.host.kak3.SHP.application.Application;
import uk.ac.st_andrews.cs.host.kak3.SHP.application.neuralnet.*;

import java.io.FileWriter;
import java.io.IOException;
import java.io.PrintWriter;
import java.util.Arrays;
import java.util.Random;
import java.util.concurrent.*;
import java.util.stream.Collectors;

/**
 * A basic implementation of the framework, simply proxies communication between algorithms and applications, and
 * instantiates the appropriate classes based on command-line parameters provided in main().
 */
public class BasicFramework implements Framework {
    private long timeout;
    private long iterationTimeout;

    protected final SearchAlgorithm algorithm;
    private final Application application;
    protected ControlParameters currentParams;

    private SampleTechnique sampler;
    private Integer snapshotAfter;
    private String snapshotPath;
    private Double snapshotGranularity;
    private int iterations = 0;

    private boolean finished = false;

    protected StatsGatherer stats;

    private void initialiseParams(boolean randomise) {
        if (randomise) {
            double[] params = new double[application.getParamCount()];
            Random random = new Random();
            for (int i = 0; i < application.getParamCount(); i++) {
                params[i] = random.nextDouble();
            }
            currentParams = new ControlParameters(params);
        }
        else {
            currentParams = application.initialise();
        }

        application.doMove(currentParams);
    }

    public BasicFramework(Namespace args) {
        if (this instanceof Application) {
            this.application = (Application) this;
        }
        else {
            NeuralProblem problem = getProblem(args.getString("problem"));
            this.application = getApplication(args.getString("application"), problem);
        }

        this.timeout = args.getLong("timeout");
        this.iterationTimeout = args.getLong("iteration_timeout");

        this.sampler = getSampleTechnique(
                args.getString("sampler"),
                args.getDouble("stepSize"),
                args.getInt("samples"),
                args.getDouble("minval"),
                args.getDouble("maxval")
        );

        this.algorithm = getAlgorithm(
                args.getString("algorithm"),
                this.sampler,
                args.getDouble("learnrate"),
                args.getBoolean("samplesonly"),
                args.getDouble("mincost"),
                args.getDouble("maxcost"),
                args.getDouble("energy_coefficient"),
                args.getDouble("temperature"),
                args.getDouble("coolingrate")
        );

        this.snapshotAfter = args.getInt("snapshot_after");
        this.snapshotPath = args.getString("snapshot_path");
        this.snapshotGranularity = args.getDouble("snapshot_granularity");

        if (args.getString("statspath") != null) {
            stats = new StatsGatherer(args.getString("statspath"));
        }

        // Randomly initialise starting parameters unless the flag specifying not to is passed, or if we're using the
        // stripe problem, which relies on exact initial weight states.
        if (args.getBoolean("no_randomise_params") ||
                (args.getString("problem") != null && args.getString("problem").equals("StripeProblem"))) {
            initialiseParams(false);
        }
        else {
            initialiseParams(true);
        }
    }

    @Override
    public void move(ControlParameters parameters) {
        currentParams = parameters;
        application.doMove(parameters);

        if (stats != null) {
            stats.addMoveCost(application.getCurrentCost());
        }
    }

    @Override
    public double getCost(ControlParameters parameters) {
        if (stats != null) {
            stats.costed();
        }
        return application.simulateMove(parameters);
    }

    @Override
    public double getSequenceCost(MoveSequence sequence) {
        if (stats != null) {
            stats.sequenceCosted();
        }
        return application.simulateMoveSequence(sequence);
    }

    @Override
    public double getCurrentCost() {
        return application.getCurrentCost();
    }

    @Override
    public void run() {
        long startTime = System.currentTimeMillis();
        while (!finished) {
            if (System.currentTimeMillis() - startTime > timeout) {
                System.out.println("\nTimed out after " + timeout + "ms.");
                algorithm.terminate(this);
            }
            else {
                iteration();
            }
        }
    }

    /**
     * Performs one iteration of search, terminating it if it takes longer than iterationTimeout ms.
     */
    @Override
    public void iteration() {
        if (stats != null) {
            stats.iteration();
        }

        // Turn our synchronous iteration method into a Future, so we can terminate it if it times out.
        ExecutorService executor = Executors.newCachedThreadPool();
        Runnable task = () -> algorithm.iteration(this);
        Future future = executor.submit(task);
        try {
            future.get(iterationTimeout, TimeUnit.MILLISECONDS);
        } catch (InterruptedException | ExecutionException ignored) {
        } catch (TimeoutException e) {
            System.out.println("Iteration timed out after: " + iterationTimeout + " ms");
        }
        finally {
            future.cancel(true); // Don't bother doing the rest of the computation if we timed out.
        }

        iterations++;

        // Optionally take a snapshot of the cost surface if requested at this iteration.
        if (snapshotAfter != null && iterations == snapshotAfter) {
            try {
                writeCostsToCSV(snapshotGranularity, snapshotPath);
            } catch (IOException e) {
                System.out.println("Failed to write cost CSV snapshot.");
                e.printStackTrace();
            }
        }
    }

    @Override
    public void finished() {
        finished = true;
        application.finished();
        stats.dump();
    }

    @Override
    public ControlParameters getCurrent() {
        return currentParams;
    }

    @Override
    public ControlParameters combineMoves(MoveSequence parameters) {
        return application.combineMoves(parameters.getParameters());
    }

    @Override
    public boolean canMoveDirectly() {
        return application.canMoveDirectly();
    }

    @Override
    public double getWind() {
        return application.getWind();
    }

    private static NeuralProblem getProblem(String problemName) {
        switch (problemName) {
            case "DigitRecognition":
                return new DigitRecognition();
            case "xor":
                return new Xor();
            case "StripeProblem":
                return new StripeProblem();
            default:
                throw new Error();
        }
    }

    private static Application getApplication(String applicationName, NeuralProblem problem) {
        switch (applicationName) {
            case "NeuralNet":
                return new NeuralApplication(problem);
            default:
                throw new Error();
        }
    }

    private static SampleTechnique getSampleTechnique(String techniqueName, double stepSize, Integer numSamples,
                                                      Double minVal, Double maxVal) {
        switch (techniqueName) {
            case "AxisBased":
                System.out.println("Step size: " + stepSize);
                return new AxisBasedSampler(stepSize, minVal, maxVal);
            case "RandomSamples":
                return new RandomSampler(numSamples, stepSize, minVal, maxVal);
            case "Static":
                return new StaticSampler(minVal, maxVal, stepSize);
            case "Forwards":
                return new ForwardsSampler(minVal, maxVal, stepSize);
            default:
                throw new Error();
        }
    }

    private static SearchAlgorithm getAlgorithm(String algorithmName, SampleTechnique technique, double learnRate,
                                                boolean onlyMoveToSamples, double minCost, double maxCost,
                                                double energyCoefficient, double temperature, double coolingRate) {
        switch (algorithmName) {
            case "GradientDescent":
                return new SteepestGradientDescent(technique, minCost, learnRate, onlyMoveToSamples);
            case "SimulatedAnnealing":
                return new SimulatedAnnealing(technique, energyCoefficient, temperature, coolingRate);
            case "FractionalProgress":
                return new FractionalProgress(technique, minCost);
            case "AStar":
                return new AStar(technique, minCost);
            case "RRT":
                return new RRT(technique, minCost, maxCost);
            default:
                throw new Error();
        }
    }

    protected static ArgumentParser getParser() {
        ArgumentParser parser = ArgumentParsers.newArgumentParser("project")
                .defaultHelp(true);

        parser.addArgument("algorithm")
                .choices("GradientDescent", "SimulatedAnnealing", "FractionalProgress", "AStar", "RRT")
                .help("The search algorithm to use.");
        parser.addArgument("sampler")
                .choices("AxisBased", "RandomSamples", "Static", "Forwards")
                .help("The sample technique to use at each step in control space.");
        parser.addArgument("stepSize")
                .type(Double.class)
                .help("The step size to use when sampling.");
        parser.addArgument("--samples")
                .type(Integer.class)
                .help("The number of samples to take for random sampling.")
                .required(false);
        parser.addArgument("--learnrate")
                .type(Double.class)
                .help("The learning rate for steepest gradient descent.")
                .setDefault(-1d)
                .required(false);
        parser.addArgument("--application")
                .choices("NeuralNet")
                .help("The application to use.")
                .required(false);
        parser.addArgument("--minval")
                .type(Double.class)
                .help("The minimum allowed value for the parameters in this application.")
                .required(false);
        parser.addArgument("--maxval")
                .type(Double.class)
                .help("The maximum allowed value for the parameters in this application.")
                .required(false);
        parser.addArgument("--samplesonly")
                .help("Indicates whether supporting algorithms should only doMove directly to the sampled points.")
                .action(Arguments.storeTrue())
                .setDefault(true);
        parser.addArgument("--problem")
                .choices("xor", "DigitRecognition", "StripeProblem")
                .help("The problem instance for the particular application.")
                .required(false);
        parser.addArgument("--no-randomise-params")
                .action(Arguments.storeTrue())
                .setDefault(false)
                .help("The system will normally initialise control parameters to random values. Passing this flag" +
                        " prevents this, useful for starting from known initial configurations/recreating results.");
        parser.addArgument("--mincost")
                .type(Double.class)
                .help("The minimum cost to use for supporting algorithms. Most will consider themselves complete" +
                        "if cost is lower than this. For gradient descent, this indicates the minimum gradient.")
                .setDefault(0.0001d);
        parser.addArgument("--maxcost")
                .type(Double.class)
                .help("The maximum allowed cost for a move for supporting algorithms")
                .setDefault(100000d);
        parser.addArgument("--energy-coefficient")
                .type(Double.class)
                .help("The energy coefficient for simulated annealing. The higher this is, the more likely it is to accept" +
                        "bad moves in a high energy state.")
                .setDefault(100000d);
        parser.addArgument("--temperature")
                .type(Double.class)
                .help("The initial temperature to use for simulated annealing.")
                .setDefault(10000d);
        parser.addArgument("--coolingrate")
                .type(Double.class)
                .help("The cooling rate for simulated annealing.")
                .setDefault(0.001d);
        parser.addArgument("--snapshot-after")
                .type(Integer.class)
                .help("Snapshot the cost information to a CSV after the given number of search iterations.")
                .required(false);
        parser.addArgument("--snapshot-path")
                .type(String.class)
                .help("Path to write CSV cost snapshot to.")
                .setDefault("H:/stats.csv")
                .required(false);
        parser.addArgument("--snapshot-granularity")
                .type(Double.class)
                .help("Granularity of cost snapshot.")
                .setDefault(0.5d)
                .required(false);
        parser.addArgument("--timeout")
                .type(Long.class)
                .help("Timeout (in ms) for the run() method. Has no effect when this is not the access point.")
                .setDefault(20000L);
        parser.addArgument("--iteration-timeout")
                .type(Long.class)
                .help("Timeout (in ms) for a single iteration of search. If no move is executed within this time, the iteration" +
                        "will be terminated, and may be retried.")
                .setDefault(5000L);
        parser.addArgument("--statspath")
                .type(String.class)
                .help("Indicates that stats should be gathered and written for this run, and where to write them to.");
        return parser;
    }

    protected static Namespace parseArgs(String[] args, ArgumentParser parser) {
        try {
            return parser.parseArgs(args);
        } catch (ArgumentParserException e) {
            parser.handleError(e);
            System.exit(1);
            return null;
        }
    }

    /**
     * Writes a CSV snapshot of the cost information along all possible moves from the current position.
     * @param granularity the granularity of the sampling.
     * @param path the path to write the CSV to.
     * @throws IOException if an IOException occurs during file handling.
     */
    private void writeCostsToCSV(double granularity, String path) throws IOException {
        PrintWriter writer = new PrintWriter(new FileWriter(path));
        String header = "";
        for (int i = 0; i < currentParams.getArray().length; i++) {
            header += i + ",";
        }
        header += "cost";
        writer.println(header);

        StaticSampler staticSampler = new StaticSampler(sampler.getMinValue(), sampler.getMaxValue(), granularity);
        for (ControlParameters parameters : staticSampler.getSamplesFrom(currentParams)) {
            String line = Arrays.stream(parameters.getArray())
                    .mapToObj(String::valueOf)
                    .collect(Collectors.joining(","));
            double cost = getCost(parameters);
            line += "," + cost;
            writer.println(line);
        }

        writer.close();
    }

    public static void main(String[] args) {
        new BasicFramework(parseArgs(args, getParser())).run();
    }
}
