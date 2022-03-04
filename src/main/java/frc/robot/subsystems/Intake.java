package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
    private static Intake instance;
    private Solenoid deploy;

    public enum IntakeState {
        IDLE, INTAKING, OUTTAKING, INDEXING,
    }

    private IntakeState state = IntakeState.IDLE;
    private final TalonFX roller;

    public Intake() {
        roller = new TalonFX(Constants.RobotMap.intake);
        roller.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, 255);
        roller.setInverted(true);
        deploy = new Solenoid(PneumaticsModuleType.CTREPCM, Constants.RobotMap.intakeDeploy);
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
        roller.set(ControlMode.PercentOutput, percentage);
    }

    public void setState(IntakeState wanted_state) {
        state = wanted_state;
    }

    @Override
    public void periodic() {

      synchronized (Intake.this) {
          switch(getState()) {
            case IDLE:
                deploy.set(false);
                roller.set(ControlMode.PercentOutput, 0.0);
                break;
            case INTAKING:
                deploy.set(true);
                roller.set(ControlMode.PercentOutput, 0.80);
                break;
            case OUTTAKING:
                deploy.set(true);
                roller.set(ControlMode.PercentOutput, -0.25);
                break;
            default:
                break;
          }
      }
    }
}
