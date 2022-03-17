// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class DisableClimbSoftLimits extends CommandBase {
  /** Creates a new DisableClimbSoftLimits. */
  public DisableClimbSoftLimits() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    RobotContainer.climber.leftClimber.configReverseSoftLimitEnable(false);
    RobotContainer.climber.leftClimber.configForwardSoftLimitEnable(false);
    RobotContainer.climber.rightClimber.configReverseSoftLimitEnable(false);
    RobotContainer.climber.rightClimber.configForwardSoftLimitEnable(false);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.climber.setClimbEncoders(0);
    RobotContainer.climber.leftClimber.configReverseSoftLimitEnable(true);
    RobotContainer.climber.leftClimber.configForwardSoftLimitEnable(true);
    RobotContainer.climber.rightClimber.configReverseSoftLimitEnable(true);
    RobotContainer.climber.rightClimber.configForwardSoftLimitEnable(true);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
