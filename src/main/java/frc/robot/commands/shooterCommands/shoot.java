package frc.robot.commands.shooterCommands;

import edu.wpi.first.wpilibj2.command.Command;
//import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.BurperSubsystem;
import frc.robot.subsystems.SlurperSubsystem;

public class shoot extends Command{

    private long tick = 0;
    private long start;
    private BurperSubsystem burper;
    //private SlurperSubsystem slurper = new SlurperSubsystem();

    public shoot(BurperSubsystem burper, SlurperSubsystem slurper) {
        this.burper = burper;
        //this.slurper = slurper;
    }


    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        start = System.currentTimeMillis();
    }
  
    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        tick = System.currentTimeMillis() - start;

        burper.setSpeed(1);

        // if(tick >= 500 && tick < 540){
        //     slurper.setSpeed(-.5);
        // }
        // if(tick >= 540){
        //     slurper.setSpeed(1);
        // }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
      burper.setSpeed(0);
      //slurper.setSpeed(0);
    }

      // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(tick >= 1000){
     return true;
    }
    return false;
  }

  
}
