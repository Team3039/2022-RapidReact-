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
    private final TalonFX mMaster;

    public Intake() {
        mMaster = new TalonFX(Constants.Ports.INTAKE);
        mMaster.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, 255);

        mMaster.setInverted(true);
        deploy = new Solenoid(PneumaticsModuleType.CTREPCM, Constants.Ports.INTAKE_SOLENOID);
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
        mMaster.set(ControlMode.PercentOutput, percentage);
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
                mMaster.set(ControlMode.PercentOutput, 0.0);
                break;
            case INTAKING:
                deploy.set(true);
                mMaster.set(ControlMode.PercentOutput, 0.80);
                break;
            case OUTTAKING:
                deploy.set(true);
                mMaster.set(ControlMode.PercentOutput, -0.25);
                break;
            default:
                break;
          }
      }
    }
}
