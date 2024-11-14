package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.Angle;
import frc.robot.subsystems.SwerveSystem;

public class SwerveCommand extends Command{
    public SwerveSystem swerve;
    public XboxController xbox;
    // public boolean apress=false,bpress=false,ypress=false,xpress=false;
    public double iLx, iLy;
    public double rightY = 0;
    public double theta;
    public double allSpeed = 0;
    public boolean aPressed = true;
    public double thetaFL = 0, thetaFR = 0, thetaBL = 0, thetaBR = 0;
    public SwerveCommand(SwerveSystem m_swerve,XboxController m_xbox){
        xbox = m_xbox;
        swerve = m_swerve;
        // anghoul = new Angle(0,0);
        swerve.changeDirectionsBackLeft();
        // swerve.changeDirectionsFrontLeft();
        swerve.zeroMotors();
        addRequirements(swerve);
    }

    //execute order 66 busters
    @Override
    public void execute() {
        rightY = -xbox.getRightY();
        if (Math.abs(xbox.getRightY()) > 0.2){
            swerve.moveAllPower(rightY*0.2,rightY*0.2,rightY*0.2,rightY*0.2);

        }
        else {
            swerve.moveAllPower(0,0,0,0);
        }
        // swerve.moveAllPower(allSpeed*0.5,allSpeed*0.5,allSpeed*0.5,allSpeed*0.5);
        iLx = xbox.getLeftX();
        iLy = -xbox.getLeftY();

        // if ((Math.abs(iLx) > 0.3 || Math.abs(iLy) > 0.3) && xbox.getXButton()){
        if (xbox.getPOV() != -1){
            // thetaFL = swerve.angleGetter(iLx,iLy);
            // thetaFR = swerve.angleGetter(iLx,iLy);
            // thetaBL = swerve.angleGetter(iLx,iLy);
            // thetaBR = swerve.angleGetter(iLx,iLy);
            thetaFL = xbox.getPOV();
            thetaFR = xbox.getPOV();
            thetaBL = xbox.getPOV();
            thetaBR = xbox.getPOV();
        }
        SmartDashboard.putNumber("angle: ", swerve.angleGetter(iLx,iLy));
        swerve.moveControlFrontLeft(thetaFL);
        swerve.moveControlFrontRight(thetaFR);
        swerve.moveControlBackLeft(thetaBL);
        swerve.moveControlBackRight(thetaBR);

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
