// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.drive.Drive;

public class VisionSubsystem extends SubsystemBase {
  private Drive m_driveSubsystem;
  /** Creates a new VisionSubsystem. */
  private final Field2d field2d = new Field2d();

  private Transform3d cameraPose =
      new Transform3d(
          new Translation3d(
              Units.inchesToMeters(-12.5), Units.inchesToMeters(11), Units.inchesToMeters(0)),
          new Rotation3d(0, 0, 0));

  public VisionSubsystem(Drive driveSubsystem) {
    m_driveSubsystem = driveSubsystem;
    ShuffleboardTab tab = Shuffleboard.getTab("Vision");
    tab.addString("Pose", this::getFomattedPose).withPosition(0, 0).withSize(4, 0);

    tab.addString("Pose2", this::getFomattedPose2).withPosition(0, 3).withSize(2, 0);
    tab.add("Field", field2d).withPosition(3, 0).withSize(6, 4);
  }

  @Override
  public void periodic() {

    boolean useMegaTag2 = !true; // set to false to use MegaTag1
    boolean doRejectUpdate = false;
    if (useMegaTag2 == false) {
      LimelightHelpers.PoseEstimate mt1 =
          LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight-one");

      if (mt1.tagCount == 1 && mt1.rawFiducials.length == 1) {
        if (mt1.rawFiducials[0].ambiguity > .7) {
          doRejectUpdate = true;
        }
        if (mt1.rawFiducials[0].distToCamera > 3) {
          doRejectUpdate = true;
        }
      }
      if (mt1.tagCount == 0) {
        doRejectUpdate = true;
      }

      if (!doRejectUpdate) {
        SmartDashboard.putString("mt1", getFomattedPose(mt1.pose));
        m_driveSubsystem.addVisionMeasurement(
            cameraTransform(mt1.pose),
            mt1.timestampSeconds,
            VecBuilder.fill(.1, .1, .1)); // 9999999));
        SmartDashboard.putNumber("mt1X", mt1.timestampSeconds);
        updateField();
      }
    } else if (useMegaTag2 == true) {
      LimelightHelpers.SetRobotOrientation(
          "limelight-one", m_driveSubsystem.getPose().getRotation().getDegrees(), 0, 0, 0, 0, 0);
      LimelightHelpers.PoseEstimate mt2 =
          LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight-one");

      if (mt2 != null) {

        if (Math.abs(m_driveSubsystem.getTurnRate())
            > 720) // if our angular velocity is greater than 720 degrees per second, ignore vision
        // updates
        {
          doRejectUpdate = true;
        }
        if (mt2.tagCount == 0) {
          doRejectUpdate = true;
        }
        if (!doRejectUpdate) {
          m_driveSubsystem.addVisionMeasurement(
              cameraTransform(mt2.pose), mt2.timestampSeconds, VecBuilder.fill(.5, .5, .5));

          updateField();
        }
      }
    }
  }

  public Pose2d cameraTransform(Pose2d pose) {
    return new Pose3d(pose).transformBy(cameraPose).toPose2d();
  }

  public void updateField() {
    field2d.setRobotPose(m_driveSubsystem.getPose());
  }

  private String getFomattedPose() {
    return getFomattedPose(m_driveSubsystem.getPose());
  }

  private String getFomattedPose2() {
    return getFomattedPose(m_driveSubsystem.getPose2());
  }

  private String getFomattedPose(Pose2d pose) {

    return String.format(
        "(%.2f, %.2f) %.2f degrees", pose.getX(), pose.getY(), pose.getRotation().getDegrees());
  }
}
