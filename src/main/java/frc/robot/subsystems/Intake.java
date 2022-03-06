package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

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
    private CANSparkMax roller;

    public Intake() {
        roller = new CANSparkMax(Constants.RobotMap.intake, MotorType.kBrushless);
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
        roller.set(percentage);
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
              roller.set(0.0);
                break;
            case INTAKING:
              deploy.set(true);
              roller.set(0.50);
                break;
            case OUTTAKING:
              deploy.set(true);
              roller.set(-0.25);
                break;
            default:
                break;
          }
      }
    }
}
