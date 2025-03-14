package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Vars.Throttles;

public class CommandHoldThrottle extends Command {
    private final double m_throttle;

    public CommandHoldThrottle(double throttle){
        m_throttle = throttle;
    
    }
    
    @Override 
    public void execute(){
        Throttles.kDriveThrottle = m_throttle;
    }

    @Override
    public void end(boolean interrupted) {
        Throttles.kDriveThrottle = Throttles.kNormal;
    }

    // @Override
    // public boolean isFinished() {
    //     return true;
    // }
}
