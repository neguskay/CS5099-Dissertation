package main.controllers.testControllers.metrics;

/**
 * Metrics Calculator class.
 * Used and instantiated in the controller to generate necessary metrics.
 */
public class MetricsCalculator {

  private long nodeGenTime;
  private long nodeDriveTime;

  private double distanceTravelled;
  private double cumulativeEuclidean;
  private double nodeEfficiency;
  private double cumulateTurnAngle;
  private double strain;

  private int nodeCount;
  private int nodeExploredCount;

  /**
   * Calculator which returns metrics within the algorithm.
   * Used after parsing the following arguments.
   * Call the getMetricsSummary() method.
   * @param nodeGenTime The total CPU run time used to generate nodes.
   * @param nodeDriveTime The total CPU run time used to execute drive commands for the robot.
   * @param distanceTravelled The total distance Travelled by the robot.
   * @param nodeCount The total number of nodes.
   * @param nodeExploredCount The total number of nodes travelled/explored.
   * @param cumulativeEuclidean The total cumulative absolute node-to-node travelled Euclidean distance.
   * @param cumulativeTurnAngle The total cumulative absolute node-to-node travelled bearing angle.
   */
  public MetricsCalculator(long nodeGenTime, long nodeDriveTime, double distanceTravelled,
                           int nodeCount, int nodeExploredCount, double cumulativeEuclidean,
                           double cumulativeTurnAngle) {

    this.nodeGenTime = nodeGenTime;
    this.nodeDriveTime = nodeDriveTime;
    this.distanceTravelled = distanceTravelled;
    this.cumulativeEuclidean = cumulativeEuclidean;
    this.nodeCount = nodeCount;
    this.nodeExploredCount = nodeExploredCount;
    this.cumulateTurnAngle = cumulativeTurnAngle;

  }

  private long avgNodetoNodeTime(){
    return avgNodeGenTime() + avgDriveExecTime();
  }

  private long avgNodeGenTime(){
    return this.nodeGenTime/this.nodeCount;
  }

  private long avgDriveExecTime(){
    return this.nodeDriveTime/this.nodeExploredCount;
  }

  private double avgDistancePerNode(){
    return this.distanceTravelled/this.nodeExploredCount;
  }

  private double avgTurnAngle(){
    return this.cumulateTurnAngle/nodeExploredCount;
  }

  private double avgEuclideanPerNode(){
    return this.cumulativeEuclidean/this.nodeExploredCount;
  }

  private double calculateNodeEfficiency(){
    return (this.nodeExploredCount*1.0/this.nodeCount) * 100;
  }

  private void calculateStrain(){
    //Own function
  }

  public void getMetricsSummary(){
    System.out.println("Average Node To Node Time :: " + avgNodetoNodeTime());
    System.out.println("Average Node Generation Time :: " + avgNodeGenTime());
    System.out.println("Average Node Drive Time :: " + avgDriveExecTime());
    System.out.println("Average Distance Travelled per Node :: " + avgDistancePerNode());
    System.out.println("Average Euclidean Distance Travelled per Node-To-Node :: " + avgEuclideanPerNode());
    System.out.println("Average Turn Angle:: " + avgTurnAngle());
    System.out.println("Node Efficiency :: " + calculateNodeEfficiency());
    System.out.println("Strain :: ");
  }
}
