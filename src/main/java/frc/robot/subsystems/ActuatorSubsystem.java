package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


public class ActuatorSubsystem extends SubsystemBase{
        /** Creates a new ShooterSubsystem. */

    private  SparkMax m_leftActuator;
    private  SparkMax m_rightActuator;
    private final SparkMaxConfig m_leftActuatorConfig;
    private final SparkMaxConfig m_rightActuatorConfig;
    //public static final IdleMode leftActuatorIdleMode = IdleMode.kBrake;
    double maxHeight = -200;
    double minHeight = 0;
    boolean toHigh = true;
    boolean toLow = false;

    public ActuatorSubsystem() {

        //m_leftActuator = new SparkMax(Constants.DriveConstants.kLeftActuatorCanId, MotorType.kBrushless);
        m_leftActuatorConfig = new SparkMaxConfig();
        m_leftActuatorConfig.idleMode(IdleMode.kBrake);
        m_leftActuator.configure(m_leftActuatorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
       
        //m_rightActuator = new SparkMax(Constants.DriveConstants.kRightActuatorCanId, MotorType.kBrushless);
        m_rightActuatorConfig = new SparkMaxConfig();
        m_rightActuator.configure(m_rightActuatorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        //m_leftActuator.idleMode(leftActuatorIdleMode); 

    }

    @Override
    public void periodic() {
    }

    public void extendRight(double speed) {
        m_rightActuator.set(speed);
    }

    public void extendLeft(double speed) {
        m_leftActuator.set(speed);
    }

    public void retractRight(double speed){
        m_rightActuator.set(-speed);
    }

    public void retractLeft(double speed){
        m_leftActuator.set(-speed);
    }
}
