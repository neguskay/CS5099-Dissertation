package main.controllers.testControllers.node;

import main.java.kak3.shp.framework.ControlParameters;
import main.java.kak3.shp.framework.Position;

import java.util.Comparator;
import java.util.List;

public class Node {

  private double x;
  private double y;
  private double cost;
  private Position position;
  private Node parent;
  private List<Node> children;
  private ControlParameters nodeParameters;


  public Node(Position position, double cost, Node parent,List<Node> children){
    this.cost = cost;
    this.position = position;
    this.parent = parent;
    this.children = children;
    this.x = position.x;
    this.y = position.y;
    this.nodeParameters = new ControlParameters(new double[]{this.x, this.y});
  }

  public double getX() {
    return x;
  }

  public void setX(double x) {
    this.x = x;
  }

  public double getY() {
    return y;
  }

  public void setY(double y) {
    this.y = y;
  }

  public Position getPosition() {
    return position;
  }

  public void setPosition(Position position) {
    this.position = position;
  }

  public double getCost() {
    return cost;
  }

  public void setCost(double cost) {
    this.cost = cost;
  }

  public Node getParent() {
    return parent;
  }

  public void setParent(Node parent) {
    this.parent = parent;
  }

  public List<Node> getChildren() {
    return children;
  }

  public void setChildren(List<Node> children) {
    this.children = children;
  }

  public ControlParameters getNodeParameters() {
    return this.nodeParameters;
  }

  public void setNodeParameters(ControlParameters nodeParameters) {
    this.nodeParameters = nodeParameters;
  }
}