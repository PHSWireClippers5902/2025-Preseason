package frc.robot.commands;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.XboxController;

import edu.wpi.first.wpilibj.Timer;

public class LimeLight extends Command{
    //!!IMPORTANT!! THIS ONLY WORKS WITH ROBOTS THAT HAVE A LIMELIGHT, but THIS SUBSYSTEM IS MOSTLY FOR GERALD, until Dan changes it. 
    //ton of initializes variables
    public MecanumSystem tank;
    private double x = 0,y=0,nums=0,area=0;
    private XboxController m_xbox;
    private Timer timer;
    private final  LimeLightValues values; 
    private int autocorrect = 0;

    public LimeLight(LimeLightValues limelightvalues, XboxController xbox, MecanumSystem meca){
        //this part is essential, it is mapping of objects to existing objects, and adding subsystem requirements. 
        m_xbox =  xbox;
        tank = meca;
        values = limelightvalues;
        addRequirements(values);
        timer = new Timer();
        timer.reset();
        timer.start();
        //addRequirements(values);
    
    }
    

    @Override
    public void execute(){
        //sets values to offsets diagramed by limelight. 
        x = values.getTx();
        y = values.getTy();
        area = values.getTa();
        nums = values.getTv();
        //post to smart dashboard periodically
        SmartDashboard.putNumber("LimelightX", x);
        SmartDashboard.putNumber("LimelightY", y);
        SmartDashboard.putNumber("LimelightArea", area);
        SmartDashboard.putNumber("DistanceFromGoal", values.getInchesFromGoal());
        SmartDashboard.putNumber("Limelight Values: ",nums);
        //base position control. 
        float Kp = -0.1f;
        float min_command = 0.05f;

        //type of driving, just like the drivewithtank command. If this is commented, then it means that the robot isn't being used. Uncomment to use. 
        if (m_xbox.getXButton()){
            //autocorrect = 2;
        }
        if (m_xbox.getYButton()){
            //autocorrect = 1;
        }
        if (m_xbox.getAButton()){
            //autocorrect = 0;
        }
        if (m_xbox.getBButton()){
            autocorrect = 2;
        }
        if (autocorrect == 1)
        {
            //gets the heading error and adjusts based on that. 
            double heading_error = -values.getTx();
            double steering_adjust = 0.0;
            //if the heading error is greater than 0.7, then it will adjust the steering. 
            if (Math.abs(heading_error) > 0.7) 
            {
                if (heading_error < 0) 
                {
                    steering_adjust = Kp*heading_error + min_command;
                } 
                else 
                {
                    steering_adjust = Kp*heading_error - min_command;
                }
            }
            //actually drives the tankdrive based on the error 
            
            //tank.drive(-0.1*steering_adjust, 0.1*steering_adjust); 
            //left_command += steering_adjust;
            //right_command -= steering_adjust;
        
            if (values.getTv() == 0){
                //tank.drive(0,0);
            }
            else if (Math.abs(heading_error) < 0.7){
                //tank.drive(0,0);
            }

            //displays the error
            SmartDashboard.putNumber("heading_error",heading_error);
        }




        if (m_xbox.getBButton()){


            //sets the error adjustment values to the PID control. 
            Kp = -0.1f;
            min_command = 0.05f;

            double heading_error = -values.getTx();
            double steering_adjust = 0.0;
            
            double sadj = 0.11;


            //HEADING ERROR CALCULATIONS


            if (Math.abs(heading_error) > 0.7) 
            {
                if (heading_error < 0) 
                {
                    steering_adjust = Kp*heading_error + min_command;
                } 
                else 
                {
                    steering_adjust = Kp*heading_error - min_command;
                }
            } 
            //left_command += steering_adjust;
            //right_command -= steering_adjust;
        
            //displays stuff, and this doubles down and gets the distance from the goal. 
            SmartDashboard.putNumber("heading_error",heading_error);
            double xdist = values.getInchesFromGoal();
            double err = 4;
            double targ = 42;
            double sp = 0;
            //double dif = Math.abs(xdist - targ);
            //double inc = 0.005;
            if (Math.abs(targ - xdist) >= err)
            {
                if (targ-xdist > 0){
                    sp = sadj;
                }
                else
                {
                    sp = -sadj;
                }
            }
            else
            {
                sp = 0;
            }
            //drives the steering to the sum of both of the adjusted movements. 
            SmartDashboard.putNumber("Steering_adjust: ", steering_adjust);
            //PART THAT MAKES STUFF MOVE
            tank.moveFL(-0.1*steering_adjust+sp);
            tank.moveFR(0.1*steering_adjust+sp);
            tank.moveBL(-0.1*steering_adjust+sp);
            tank.moveBR(0.1*steering_adjust+sp);
            //tank.drive, 0.1*steering_adjust+sp);
            if (values.getTv() == 0){
                // tank.drive(0,0);
                tank.moveFL(0);
                tank.moveFR(0);
                tank.moveBL(0);
                tank.moveBR(0);
            }
            

            
            

        }
        
    }
}
