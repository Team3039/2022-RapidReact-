// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class SetClimbSpeed extends CommandBase {
  /** Creates a new SetClimbSpeed. */
  double percent;
  public SetClimbSpeed(double percent) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.climber);
    this.percent = percent;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    System.out.println("Bruh");
    RobotContainer.climber.setClimberOutput(percent);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.climber.setClimberOutput(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
