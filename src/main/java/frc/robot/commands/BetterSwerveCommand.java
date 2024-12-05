package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Swerve;

public class BetterSwerveCommand extends Command {
    public Joystick myJoystick;
    public Swerve mySwerve;
    private static final double kDeadband = 0.2;
    private static final double kZedDeadBand = 0.4;
    private static final double kMaxSpeed = Swerve.kMaxSpeed;
    private static final double kMaxAngularSpeed = Swerve.kMaxAngularSpeed;
    double xSpeed,ySpeed,rot;
    public BetterSwerveCommand(Joystick js, Swerve m_s){
        myJoystick = js;
        mySwerve = m_s;
        addRequirements(mySwerve);
    }
    @Override
    public void execute() {
        

        boolean fieldRelative = false;
        // if (myJoystick.getRawButton(0)){

        // }

        if (myJoystick.getRawButton(1)){

        }
        else {
            xSpeed = -applyDeadband(myJoystick.getY()) * kMaxSpeed*0.3;
            // double ySpeed = 0,rot = 0;
            // double rot = 0;
            ySpeed = applyDeadband(myJoystick.getX()) * kMaxSpeed*0.3;
            rot = -applyZedDeadband(myJoystick.getZ()) * kMaxAngularSpeed*0.3;
        }
        mySwerve.drive(xSpeed,ySpeed,rot,fieldRelative,0.02);

    }
    private double applyDeadband(double value) {
        return Math.abs(value) > kDeadband ? value : 0.0;
    }
    private double applyZedDeadband(double value){
        return Math.abs(value) > kZedDeadBand ? value: 0.0;
    }

}
