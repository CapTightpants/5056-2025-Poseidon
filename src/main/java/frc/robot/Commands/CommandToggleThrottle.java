package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Vars.Throttles;

public class CommandToggleThrottle extends Command {
    private final double m_throttle;

    public CommandToggleThrottle(double throttle){
        m_throttle = throttle;
    
    }
    
    @Override 
    public void execute(){
        Throttles.kDriveThrottle = m_throttle;
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
