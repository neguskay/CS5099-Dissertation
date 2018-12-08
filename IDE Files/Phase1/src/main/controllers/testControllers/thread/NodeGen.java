package main.controllers.testControllers.thread;

import main.controllers.testControllers.simulatedAnnealing.TestSimulated;

public class NodeGen extends Thread{
  public TestSimulated simController;

  public NodeGen(TestSimulated controller){
    this.simController = controller;
  }

  public void run(){
    simController.initAlgorithm();
  }
}

