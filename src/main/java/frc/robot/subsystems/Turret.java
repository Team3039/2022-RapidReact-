// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotMap;


public class Turret extends SubsystemBase {

  public enum TurretMode {
    TRACKING, 
    DRIVE,
    LEFT,
    RIGHT
  }
  
  public double turretAngle;
  private TurretMode turretmode = TurretMode.DRIVE;

  TalonSRX turret = new TalonSRX(RobotMap.turret);

  public static double targetValid; //Whether the limelight has any valid targets (0 or 1)
  public static double targetX; //HorizontalDR Offset From Crosshair To Target (-27 degrees to 27 degrees
  public static double targetY; //Vertical Offset From Crosshair To Target (-20.5 degrees to 20.5 degrees)
  public static double targetArea; //Target Area (0% of image to 100% of image)


  /** Creates a new Turret. */
  public Turret() {
    
  }
   
   public void setTurretMode(TurretMode Mode) {
     this.turretmode = Mode;
   }
   
   public TurretMode getTurretMode() {
     return turretmode;
   }

   public void setCamMode(boolean setVision) {
     if(setVision) {
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("camMode").setNumber(0);
   }
     else {
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("camMode").setNumber(1);   
  }
  }

   public void setLedMode(boolean setLED) {
     if(setLED) {
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(3);
   }
     else {
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(1);
   }
     }


   
    public void trackTarget() {
      double errorX = targetX % 360;
      double correctionX = errorX * Constants.kP_TURRET_TRACK;

     // SmartDashboard.putNumber("turretOutput", correctionX);
     // SmartDashboard.putNumber("targetX", targetX);

      if(turret.getSelectedSensorPosition() >= 90 && targetX < 0) {
        turret.set(ControlMode.PercentOutput, 0);
      } 
      else if(turret.getSelectedSensorPosition() <= -90 && targetX > 0) {
        turret.set(ControlMode.PercentOutput, 0);
      }
      else {
      turret.set(ControlMode.PercentOutput, correctionX);
     }
    }

    public void manualTarget(double targetAngle) {
  
    }

    public void stop() {
      turret.set(ControlMode.PercentOutput, 0);
      setCamMode(false);
      
    }
  //rotate turret to specified angle
    public void targetPose(double targetAngle) {
      double errorX = turret.getSelectedSensorPosition() - targetAngle;
      double correctionX = MathUtil.clamp(errorX * Constants.kP_TURRET_RESET_POS, -0.7, 0.7);

    //  SmartDashboard.putNumber("TurretEncoder", turret.getSelectedSensorPosition());
    //  SmartDashboard.putNumber("TargetPosOutput", correctionX);

      turret.set(ControlMode.PercentOutput, correctionX);
    }


  @Override
  public void periodic() { 
    // This method will be called once per scheduler run
    targetValid = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0);
    targetX = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);
    targetY = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0);
    targetArea = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ta").getDouble(0);

    SmartDashboard.putNumber("TurretEncoder", turret.getSelectedSensorPosition());
    
    TurretMode currentMode = getTurretMode();
    switch(currentMode) {
      case TRACKING:
         setCamMode(true);
         trackTarget();
         break;
      case DRIVE:
         setCamMode(false);
         targetPose(turretAngle);
         break;
      default:
         stop();
    }
  } 
}
