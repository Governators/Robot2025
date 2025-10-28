// package frc.robot.commands.intakeCommands;

// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.subsystems.SlurperSubsystem;

// public class startIntake extends Command{


//     private SlurperSubsystem slurper;
//     private long tick = 0;
//     private long start;

//     //This is the constructor class gets called back in robotContainer when it is 
//     public startIntake(SlurperSubsystem slurper) {
//         this.slurper = slurper;
//     }

//     // Called when the command is initially scheduled.
//     @Override
//     public void initialize() {
  
//     }
  
//     // Called every time the scheduler runs while the command is scheduled.
//     @Override
//     public void execute() {

//       slurper.setSpeed(.5); 

//     }

//     // Called once the command ends or is interrupted.
//     @Override
//     public void end(boolean interrupted) {
//       slurper.setSpeed(.5);
//     }
  
//     // Returns true when the command should end.
//     @Override
//     public boolean isFinished() {
//       return true;
//     }
// }
