package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Swerve;

public class BetterSwerveCommand extends Command {
    public Joystick myJoystick;
    public Swerve mySwerve;
    private static final double kDeadband = 0.2;
    private static final double kMaxSpeed = Swerve.kMaxSpeed;
    private static final double kMaxAngularSpeed = Swerve.kMaxAngularSpeed;

    public BetterSwerveCommand(Joystick js, Swerve m_s){
        myJoystick = js;
        mySwerve = m_s;
        addRequirements(mySwerve);
    }
    @Override
    public void execute() {
        double xSpeed = -applyDeadband(myJoystick.getY()) * kMaxSpeed;
        // double ySpeed = 0,rot = 0;
        // double rot = 0;
        double ySpeed = -applyDeadband(myJoystick.getX()) * kMaxSpeed;
        double rot = applyDeadband(myJoystick.getZ()) * kMaxAngularSpeed;

        boolean fieldRelative = false;

        mySwerve.drive(xSpeed,ySpeed,rot,fieldRelative,0.02);

    }
    private double applyDeadband(double value) {
        return Math.abs(value) > kDeadband ? value : 0.0;
    }

}
