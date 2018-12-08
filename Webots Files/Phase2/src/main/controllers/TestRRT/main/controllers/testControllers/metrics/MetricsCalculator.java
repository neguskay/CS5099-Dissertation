package main.controllers.testControllers.metrics;

public class MetricsCalculator {

  private long nodeGenTime;
  private long nodeDriveTime;

  private double distanceTravelled;
  private double cumulativeEuclidean;
  private int nodeCount;
  private int nodeExploredCount;
  private double nodeEfficiency;
  private double cumulateTurnAngle;

  private double strain;

  public MetricsCalculator(long nodeGenTime, long nodeDriveTime, double distanceTravelled, int nodeCount,
                           int nodeExploredCount, double cumulativeEuclidean, double cumulativeTurnAngle) {
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
    return this.cumulateTurnAngle/this.nodeExploredCount;
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
    System.out.println("Average Turn Angle:: " + avgTurnAngle());  //will include later
    System.out.println("Node Efficiency :: " + calculateNodeEfficiency());
    System.out.println("Strain :: ");
  }
}
