package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.BMotor;
import frc.robot.Constants;

//WILL HANDLE CORAL SPINNING
public class BurperSubsystem extends SubsystemBase{
        /** Creates a new ShooterSubsystem. */

    //private final BMotor RightCoralSpinner;
    //private final BMotor LeftCoralSpinner;

    private final SparkMax leftCoralSpinner; //= new SparkMax(Constants.DriveConstants.kLeftCoralSpinnerCanId, MotorType.kBrushless);
    private  final SparkMax rightCoralSpinner; // = new SparkMax(Constants.DriveConstants.kRightCoralSpinnerCanId, MotorType.kBrushless);

    public BurperSubsystem() {
      /*   RightCoralSpinner = new BMotor(Constants.DriveConstants.kRightCoralSpinnerCanId);
        LeftCoralSpinner = new BMotor(Constants.DriveConstants.kLeftCoralSpinnerCanId);*/
        leftCoralSpinner = new SparkMax(Constants.DriveConstants.kLeftCoralSpinnerCanId, MotorType.kBrushless);
        rightCoralSpinner = new SparkMax(Constants.DriveConstants.kRightCoralSpinnerCanId, MotorType.kBrushless);
    }

    @Override
    public void periodic() {
    // This method will be called once per scheduler run
    }

    public void setSpeed(double speed) {
        leftCoralSpinner.set(speed);
        rightCoralSpinner.set(-speed);
    }

    public void setSpeedLeft(double speed){
        leftCoralSpinner.set(speed);
    }

    public void setSpeedRight(double speed){
        rightCoralSpinner.set(speed);
    }

}
