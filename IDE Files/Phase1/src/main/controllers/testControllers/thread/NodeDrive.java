package main.controllers.testControllers.thread;


import main.controllers.testControllers.simulatedAnnealing.TestSimulated;

public class NodeDrive extends Thread{

  public TestSimulated simController;

  public NodeDrive(TestSimulated controller){
    this.simController = controller;
  }

  public void run(){
    simController.initAlgorithm();
  }
}
