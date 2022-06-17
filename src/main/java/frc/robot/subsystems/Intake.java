package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class Intake extends SubsystemBase {
  private static Intake instance;
  private Solenoid deploy;

  public enum IntakeState {
    IDLE, INTAKING, OUTTAKING, CLIMBING
  }

  private IntakeState state = IntakeState.IDLE;
  private CANSparkMax roller;

  public Intake() {
    roller = new CANSparkMax(Constants.Ports.INTAKE, MotorType.kBrushless);
    deploy = new Solenoid(PneumaticsModuleType.CTREPCM, Constants.Ports.INTAKE_SOLENOID);

    roller.setControlFramePeriodMs(10);

    roller.setIdleMode(IdleMode.kCoast);
  }

  public synchronized static Intake getInstance() {
    if (instance == null) {
      instance = new Intake();
    }
    return instance;
  }

  public synchronized IntakeState getState() {
    return state;
  }

  public void setOpenLoop(double percentage) {
    roller.set(percentage);
  }

  public void setState(IntakeState wanted_state) {
    state = wanted_state;
  }

  @Override
  public void periodic() {
  //  SmartDashboard.putNumber("Intake Amps", roller.getOutputCurrent());
    // SmartDashboard.putString("Intake State", String.valueOf(getState()));

    synchronized (Intake.this) {
      switch (getState()) {
        case IDLE:
          deploy.set(false);
          roller.set(0.0);
          break;
        case INTAKING:
          if (!RobotContainer.indexer.hasTwoBalls) {
            deploy.set(!RobotContainer.indexer.isFeeding);
            roller.set(0.55);
          } else {
            deploy.set(false);
            roller.set(0.0);
          }
          break;
        case OUTTAKING:
          // deploy.set(!RobotContainer.indexer.isFeeding);
          roller.set(-0.45);
          break;
        case CLIMBING:
          roller.disable();
          break;
        default:
          break;
      }
    }
  }
}
