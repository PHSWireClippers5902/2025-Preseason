package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Angle;
import frc.robot.subsystems.SwerveSystem;

public class SwerveCommand extends Command{
    public SwerveSystem swerve;
    public XboxController xbox;
    // public boolean apress=false,bpress=false,ypress=false,xpress=false;
    public double iLx, iLy,highonpot;
    public double theta;
    public double allSpeed = 0;
    public Angle anghoul;
    public boolean aPressed = true;
    public SwerveCommand(SwerveSystem m_swerve,XboxController m_xbox){
        xbox = m_xbox;
        swerve = m_swerve;
        // anghoul = new Angle(0,0);
        System.out.println("INITIAL POSITIONS " + swerve.getControllerPositions());
        
        swerve.zeroMotors();
        System.out.println("INITIAL POSITIONS AFTER ZEROING " + swerve.getControllerPositions());
        
        addRequirements(swerve);
    }

    //execute order 66 busters
    @Override
    public void execute() {
        allSpeed = (Math.abs(xbox.getRightY())> 0.1) ? xbox.getRightY() : 0;
        swerve.moveAllPower(allSpeed*0.5,allSpeed*0.5,allSpeed*0.5,allSpeed*0.5);
        SmartDashboard.putString("Positions: ", swerve.getControllerPositions());
        if (xbox.getPOV() != -1){

        
            swerve.moveAllControl(xbox.getPOV(),xbox.getPOV(),xbox.getPOV(),xbox.getPOV());
        }
        else {
            swerve.moveAllControl(0,0,0,0);
        }
        if (xbox.getAButton()){
            if (aPressed){
                swerve.zeroMotors();
            }
            aPressed = false;
        }
        else {
            aPressed = true;
        }
        
    }
}
