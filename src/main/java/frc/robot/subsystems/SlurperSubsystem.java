// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
 
// package frc.robot.subsystems;

// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.robot.BMotor;
// import frc.robot.Constants;

/*public class SlurperSubsystem extends SubsystemBase {
  /** Creates a new IntakeSubsystem. */

//     BMotor m_rightIntake;
//     BMotor m_leftIntake;
//     boolean justRan = false;
//     int backwordsCount = 10;


//   public SlurperSubsystem() {

//     m_leftIntake = new BMotor(Constants.DriveConstants.m_LeftIntakeCanId);
//     m_rightIntake = new BMotor(Constants.DriveConstants.m_RightIntakeCanId);

//     if(justRan){
//       m_leftIntake.setSpeed(1);
//       m_rightIntake.setSpeed(-1);
//       backwordsCount -= 1;
//     }
//     if(backwordsCount == 0){
//       backwordsCount = 10;
//       justRan = false;
//     }

//   }

//   @Override
//   public void periodic() {
//     // This method will be called once per scheduler run
//   }

//   public void Slurp(double speed) {
//     m_leftIntake.setSpeed(-speed);
//     m_rightIntake.setSpeed(speed);
//     if((!justRan) && speed != 0){
//       justRan = true;
//     }

//   }

//   public void Burp(double speed){
//     speed = -speed;
//     m_leftIntake.setSpeed(speed);
//     m_rightIntake.setSpeed(speed);
//   }
// }


// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.BMotor;
import frc.robot.Constants;

//WILL HANDLE ALGAE SPINNDING
public class SlurperSubsystem extends SubsystemBase {
    /** Creates a new IntakeSubsystem. */

    // private final SparkMax leftAlgaeSpinner; //= new SparkMax(Constants.DriveConstants.kLeftCoralSpinnerCanId, MotorType.kBrushless);
    // private  final SparkMax rightAlgaeSpinner;

    // public SlurperSubsystem() {
    //     leftAlgaeSpinner = new SparkMax(Constants.DriveConstants.kLeftAlgaeSpinnerCanId, MotorType.kBrushless);
    //     rightAlgaeSpinner = new SparkMax(Constants.DriveConstants.kRightAlgaeSpinnerCanId, MotorType.kBrushless);
    // }

    // @Override
    // public void periodic() {
    //     // This method will be called once per scheduler run
    // }

    // public void setSpeed(double speed) {
    //     leftAlgaeSpinner.set(speed);
    //     rightAlgaeSpinner.set(-speed);
    // }

    /*public void Burp(double speed){
        speed = -speed;
        leftAlgaeSpinner.set(speed);
        }

    public void Slurp(double speed){
        rightAlgaeSpinner.set(-speed);
        }*/
    }
