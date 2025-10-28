// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


package frc.robot;


import edu.wpi.first.math.MathUtil;

import edu.wpi.first.wpilibj.XboxController;
//import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.PS4Controller.Button;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.OIConstants;
// import frc.robot.commands.intakeCommands.startIntake;
import frc.robot.commands.intakeCommands.stopIntake;
import frc.robot.commands.shooterCommands.shoot;
import frc.robot.subsystems.BurperSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.SlurperSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
//import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
//import java.util.Map;

import com.pathplanner.lib.auto.AutoBuilder;
//import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;


/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {


    //Creates an autoChoser idk wtf an autoChooser is but thats what it does I think we need one?
    /*
    Update still dont realy know wtf an autoChooser is but I know that it can communicate with the smart dashboard which can let
    you select what auto you want to use if you are smart and have multible autos than thats what this is for im leaving it here for 
    you or me if either of us decides that is something we are smart enough for

    If I had to guess I would guess that it selects an auto -Joel

    Well yes but how tf it does that is still unknow rookie -Jonah

    cough cough "robot object" - Joel

    (quiet crying in the background) - Jonah
    */
    // private final SendableChooser<Command> autoChooser;



  //BurperSubsystem burper = new BurperSubsystem();
  // The robot's subsystems
  private final DriveSubsystem m_robotDrive;

  // The driver's controller
  XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);
  // XboxController m_shootController = new XboxController(OIConstants.kShootControllerPort);


  //subSystems
  // BurperSubsysstem buper = new BurperSubsystem();
  //SlurperSubsystem slurper = new SlurperSubsystem();


  //private final SendableChooser<Command> autoChooser;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public void configureAutoCommands(){

    /*
     * Here are our auto commands if you want to creat another auto command to make your auto do stuff 
     * you first need to create another command which I have helpfully created a command folder for
     * go into the command folder copy an existing command and paste it into a new file
     * it should be self explanitory from there
     * All our auto/command stuff is stolen from 3939 https://github.com/frc-team3939/2024-RobotCode/blob/main/2024-RobotCode/src/main/java/frc/robot/RobotContainer.java
     * 
     */
    //Intake Commands

    //JOEL DID THE SPEED TO .5 
    /*NamedCommands.registerCommand("startIntake", new startIntake(slurper));
    //just the one above this tho
    NamedCommands.registerCommand("shoot", new shoot(buper, slurper));
    // NamedCommands.registerCommand("stopShooter", new shoot(buper, slurper, 0));
    NamedCommands.registerCommand("stopIntake", new stopIntake(slurper));*/

  }





  public RobotContainer() {

    configureAutoCommands();
    System.out.println("RobotContainer");

    // autoChooser = AutoBuilder.buildAutoChooser(auto);

    //autoChooser = AutoBuilder.buildAutoChooser();
    //SmartDashboard.putData(autoChooser);

    m_robotDrive = new DriveSubsystem();

    //configureButtonBindings();

    m_robotDrive.setDefaultCommand(
      // The left stick controls translation of the robot.
      // Turning is controlled by the X axis of the right stick.
      new RunCommand(
          () -> m_robotDrive.drive(
              -MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDriveDeadband),
              -MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDriveDeadband),
              -MathUtil.applyDeadband(m_driverController.getRightX(), OIConstants.kDriveDeadband),
              true, true),
          m_robotDrive));



  }


    /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its
   * subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling
   * passing it to a
   * {@link JoystickButton}.
   */
  // private void configureButtonBindings() {
  //   System.out.println("Configure button Bindings works");
  //   new JoystickButton(m_driverController, Button.kR1.value)
  //       .whileTrue(new RunCommand(
  //           () -> m_robotDrive.setX(),
  //           m_robotDrive));
  // }



  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return new PathPlannerAuto("Blue");
    //return autoChooser.getSelected();
  }
}