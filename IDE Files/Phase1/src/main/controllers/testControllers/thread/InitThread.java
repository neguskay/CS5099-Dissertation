package main.controllers.testControllers.thread;

import main.controllers.testControllers.simulatedAnnealing.TestSimulated;

public class InitThread extends Thread{
  public TestSimulated simController;

  public InitThread(TestSimulated controller){
    this.simController = controller;
  }

  public void run(){
    simController.initAlgorithm();
  }
}

