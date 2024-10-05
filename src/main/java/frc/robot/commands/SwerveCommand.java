package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSystem;

public class SwerveCommand extends Command{
    public SwerveSystem swerve;
    public XboxController xbox;
    public SwerveCommand(SwerveSystem m_swerve,XboxController m_xbox){
        xbox = m_xbox;
        swerve = m_swerve;
        addRequirements(swerve);
    }
    @Override
    public void execute() {
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
        SmartDashboard.putNumber("Degrees-Input: ", (xbox.getLeftX() + 1)* 180);
        swerve.moveControlOne((xbox.getLeftX() + 1)*180);
    }
}
