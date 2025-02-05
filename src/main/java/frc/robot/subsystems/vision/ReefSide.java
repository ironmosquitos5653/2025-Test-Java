package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;

public class ReefSide {
  private Pose2d sidePosition;
  private Pose2d leftPosition;
  private Pose2d rightPosition;
  private String description;

  public ReefSide(
      Pose2d sidePosition, Pose2d leftPosition, Pose2d rightPosition, String description) {
    this.sidePosition = sidePosition;
    this.leftPosition = leftPosition;
    this.rightPosition = rightPosition;
    this.description = description;
  }

  public Pose2d getSidePosition() {
    return sidePosition;
  }

  public Pose2d getLeftPosition() {
    return leftPosition;
  }

  public Pose2d getRightPosition() {
    return rightPosition;
  }

  public String getDescription() {
    return description;
  }
}
