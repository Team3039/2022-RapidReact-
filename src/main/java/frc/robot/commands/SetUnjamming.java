// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Indexer.IndexerState;
import frc.robot.subsystems.Intake.IntakeState;
import frc.robot.subsystems.Superstructure.SuperstructureState;

public class SetUnjamming extends CommandBase {
  /** Creates a new SetUnjamming. */
  public SetUnjamming() {
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Superstructure.getInstance().setState(SuperstructureState.UNJAMMING);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Superstructure.getInstance().setState(SuperstructureState.IDLE);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
