package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Angle;
import frc.robot.subsystems.SwerveSystem;

public class SwerveCommand extends Command{
    public SwerveSystem swerve;
    public XboxController xbox;
    public double iLx, iLy,highonpot;
    public double theta;
    public Angle anghoul;
    public SwerveCommand(SwerveSystem m_swerve,XboxController m_xbox){
        xbox = m_xbox;
        swerve = m_swerve;
        anghoul = new Angle(0,0);
        addRequirements(swerve);
    }

    //execute order 66 busters
    @Override
    public void execute() {
        if (Math.abs(xbox.getLeftX()) > 0.2 ){
            iLx = xbox.getLeftX();
        }
        else {
            iLx = 0;
        }
        if (Math.abs(xbox.getLeftY()) > 0.2 ){
            iLy = -xbox.getLeftY();
        }
        else {
            iLy = 0;
        }

        anghoul.setBoth(iLx,iLy);
        if (xbox.getLeftTriggerAxis() > 0.2){
            swerve.movePowerOne(0.3*xbox.getLeftTriggerAxis());
        }
        else if (xbox.getRightTriggerAxis() > 0.2){
            swerve.movePowerOne(-0.3*xbox.getRightTriggerAxis());
        }
        else {
            swerve.movePowerOne(0);
        }
        SmartDashboard.putNumber("Motor Position: ",swerve.getControlOnePosition());
        SmartDashboard.putNumber("ControlOneRotations: ",swerve.getControlOneRotations());
        SmartDashboard.putNumber(" X: ", (iLx));
        SmartDashboard.putNumber(" Y: ", (iLy));
        SmartDashboard.putNumber("Theta",anghoul.calcAngle());
        theta = anghoul.calcAngle();
        swerve.moveControlOne(theta);
        SmartDashboard.putNumber("Percent Error: ",(theta*4096/360) / swerve.getControlOnePosition());
        // swerve.moveControlOne(360);
    }
}
