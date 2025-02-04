// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.TrajectoryCommandFactory;
import frc.robot.subsystems.drive.Drive;
import java.util.ArrayList;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class TestDriveToPosition extends Command {
  /** Creates a new TestDriveToPosition. */
  Drive m_driveSubsystem;

  TrajectoryCommandFactory m_trajectoryCommandFactory;

  public TestDriveToPosition(
      Drive driveSubsystem, TrajectoryCommandFactory trajectoryCommandFactory) {

    m_driveSubsystem = driveSubsystem;
    m_trajectoryCommandFactory = trajectoryCommandFactory;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  public Command buildTrajectoryCommand(Pose2d current, Pose2d target) {
    Trajectory trajectory =
        m_trajectoryCommandFactory.createTrajectory(
            current, new ArrayList<Translation2d>(), target);

    return m_trajectoryCommandFactory.createTrajectoryCommand(trajectory);
  }
}
