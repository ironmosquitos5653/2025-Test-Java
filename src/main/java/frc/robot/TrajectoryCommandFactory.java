package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.vision.VisionSubsystem;
import java.util.ArrayList;
import java.util.List;

public class TrajectoryCommandFactory {

  public static final double kMaxSpeedMetersPerSecond = 4;
  public static final double kMaxAccelerationMetersPerSecondSquared = 3;
  public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
  public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

  public static final double kPXController = 1;
  public static final double kPYController = 1;
  public static final double kPThetaController = 4;

  // Constraint for the motion profiled robot angle controller
  public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
      new TrapezoidProfile.Constraints(
          kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);

  private final ProfiledPIDController thetaController =
      new ProfiledPIDController(kPThetaController, 0, 0, kThetaControllerConstraints);

  private final TrajectoryConfig config;

  private Drive m_robotDrive;
  private VisionSubsystem m_VisionSubsystem;

  public TrajectoryCommandFactory(Drive robotDrive, VisionSubsystem visionSubsystem) {
    m_robotDrive = robotDrive;
    m_VisionSubsystem = visionSubsystem;
    config =
        new TrajectoryConfig(kMaxSpeedMetersPerSecond, kMaxAccelerationMetersPerSecondSquared)
            // Add kinematics to ensure max speed is actually obeyed
            .setKinematics(m_robotDrive.kinematics);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);
  }

  public Trajectory createTrajectory(
      Pose2d start, List<Translation2d> interiorWaypoints, Pose2d end) {
    return TrajectoryGenerator.generateTrajectory(start, interiorWaypoints, end, config);
  }

  public SwerveControllerCommand createTrajectoryCommand(Trajectory trajectory) {
    return new SwerveControllerCommand(
        trajectory,
        m_robotDrive::getPose, // Functional interface to feed supplier
        m_robotDrive.kinematics,

        // Position controllers
        new PIDController(kPXController, .1, 0),
        new PIDController(kPYController, .1, 0),
        thetaController,
        m_robotDrive::setModuleStates,
        m_robotDrive);
  }

  public Command getTheAwesomestTrajectoryCommand(Pose2d target) {
    Pose2d current = m_robotDrive.getPose();
    Trajectory trajectory = createTrajectory(current, new ArrayList<Translation2d>(), target);

    return createTrajectoryCommand(trajectory);
  }
}
