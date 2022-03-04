// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
<<<<<<< HEAD:src/main/java/frc/robot/commands/ShootCargo.java
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Superstructure.SuperstructureState;
=======
import frc.robot.RobotContainer;
import frc.robot.subsystems.Turret.TurretMode;
>>>>>>> parent of 4fcc90b (-Minor Edits):src/main/java/frc/robot/commands/SetTurretPos.java

public class ShootCargo extends CommandBase {
  /** Creates a new ShootCargo. */
  public ShootCargo() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
<<<<<<< HEAD:src/main/java/frc/robot/commands/ShootCargo.java
    // Superstructure.getInstance().setState(SuperstructureState.SHOOTING);
=======
    RobotContainer.turret.turretAngle = angle;
    RobotContainer.turret.setTurretMode(TurretMode.DRIVE);
>>>>>>> parent of 4fcc90b (-Minor Edits):src/main/java/frc/robot/commands/SetTurretPos.java
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // Superstructure.getInstance().setState(SuperstructureState.IDLE);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
