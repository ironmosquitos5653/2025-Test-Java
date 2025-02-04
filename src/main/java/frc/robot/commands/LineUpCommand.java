// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.TrajectoryCommandFactory;
import frc.robot.subsystems.vision.ReefSide;
import frc.robot.subsystems.vision.VisionSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class LineUpCommand extends Command {
  private Command subCommand;
  private TrajectoryCommandFactory m_TrajectoryCommandFactory;
  private VisionSubsystem m_visionSubsystem;
  private boolean m_isLeft = false;

  public LineUpCommand(
      TrajectoryCommandFactory trajectoryCommandFactory,
      VisionSubsystem visionSubsystem,
      boolean isLeft) {
    m_TrajectoryCommandFactory = trajectoryCommandFactory;
    m_isLeft = isLeft;
    m_visionSubsystem = visionSubsystem;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    ReefSide reefSide = m_visionSubsystem.findClosest();
    if (m_isLeft) {
      subCommand =
          m_TrajectoryCommandFactory.getTheAwesomestTrajectoryCommand(reefSide.getLeftPosition());
    } else {
      subCommand =
          m_TrajectoryCommandFactory.getTheAwesomestTrajectoryCommand(reefSide.getRightPosition());
    }
    subCommand.initialize();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    subCommand.execute();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    subCommand.end(interrupted);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return subCommand.isFinished();
  }
}
