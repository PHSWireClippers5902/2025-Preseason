package frc.robot.commands;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.FlyWheelAndHook;
import frc.robot.subsystems.LimeLightValues;
import frc.robot.subsystems.MecanumSystem;

public class AutonomousCommand extends Command{
    private Timer time = new Timer();
    private LimeLightValues values;
    private MecanumSystem drive;
    private FlyWheelAndHook flyhook;
    private Timer shootTimer = new Timer();
    public AutonomousCommand(MecanumSystem mecanum,FlyWheelAndHook flysystem,LimeLightValues m_limelight){
        drive = mecanum;
        values = m_limelight;
        flyhook = flysystem;
        shootTimer.reset();
        shootTimer.start();
        addRequirements(values);
        addRequirements(drive);
        addRequirements(flyhook);
    }
    
    @Override
    public void initialize() {
        time.reset();
        time.start();
        shootTimer.reset();
        shootTimer.start();
        flyhook.moveFly(0);
        //drive.autonomousConfigs();
        drive.zeromotors();
        System.out.println("Hello, World, AutoActive");
    
    }

    @Override
    public void execute() {
        //30000 per 360
        //30000/4 = 7500
        //drive.customMoveFLTo(10000);
        if (time.get() < 4){





            //WARNING: IF OTHER THAN CENTER COMMENT OUT

            //  double Kp = -0.1f;
            // double min_command = 0.05f;

            // double heading_error = -values.getTx();
            // double steering_adjust = 0.0;
            
            // double sadj = 0.11;


            // //HEADING ERROR CALCULATIONS


            // if (Math.abs(heading_error) > 0.7) 
            // {
            //     if (heading_error < 0) 
            //     {
            //         steering_adjust = Kp*heading_error + min_command;
            //     } 
            //     else 
            //     {
            //         steering_adjust = Kp*heading_error - min_command;
            //     }
            // } 
            // //left_command += steering_adjust;
            // //right_command -= steering_adjust;
        
            // //displays stuff, and this doubles down and gets the distance from the goal. 
            // SmartDashboard.putNumber("heading_error",heading_error);
            // double xdist = values.getInchesFromGoal();
            // double err = 4;
            // double targ = 42;
            // double sp = 0;
            // //double dif = Math.abs(xdist - targ);
            // //double inc = 0.005;
            // if (Math.abs(targ - xdist) >= err)
            // {
            //     if (targ-xdist > 0){
            //         sp = sadj;
            //     }
            //     else
            //     {
            //         sp = -sadj;
            //     }
            // }
            // else
            // {
            //     sp = 0;
            // }
            // //drives the steering to the sum of both of the adjusted movements. 
            // SmartDashboard.putNumber("Steering_adjust: ", steering_adjust);
            // //PART THAT MAKES STUFF MOVE
            // drive.moveFL(-0.1*steering_adjust+sp);
            // drive.moveFR(0.1*steering_adjust+sp);
            // drive.moveBL(-0.1*steering_adjust+sp);
            // drive.moveBR(0.1*steering_adjust+sp);
            // //tank.drive, 0.1*steering_adjust+sp);
            // if (values.getTv() == 0){
            //     // tank.drive(0,0);
            //     drive.moveFL(0);
            //     drive.moveFR(0);
            //     drive.moveBL(0);
            //     drive.moveBR(0);
            // }




            shootTimer.reset();
            //shootTimer.start();
            shootTimer.start();
        }

        else if (time.get() > 4 && time.get() < 6){
            drive.moveFL(0);
            drive.moveFR(0);
            drive.moveBL(0);
            drive.moveBR(0);
            if (shootTimer.get() < 1.25){
                flyhook.moveTopFlyWheel(1);
            }
            else {
                flyhook.moveTopFlyWheel(1);
                flyhook.moveBottomFlyWheel(1);
            }
            
        }
        else if (time.get() > 6 && time.get() < 6.1){
            drive.zeromotors();
            flyhook.moveFly(0);
        }
        else if (time.get() > 6.1 && time.get() < 12){
            drive.moveAllFourTo(-4,-4,-4,-4);
        }
        else if (time.get() > 12 && time.get() < 12.1){
            drive.zeromotors();
        }
        // else if (time.get() > 9.1 && time.get() < 12){
        //     //drive.moveAllFourToDegrees(180, 180, 180, 180);
        // }
        // else if (time.get() > 12 && time.get() < 12.1){
        //     drive.zeromotors();
        // }
        else {
            drive.moveAllFourTo(0,0,0,0);
        }
        
    



       
            
        // else if (time.get() > 3.1 && time.get() < 5){
        //     //drive.moveAllFourToTURNS(8,-8,8,-8);
        //     //drive.moveAllFourToDegrees(90,90,90,90);
        //     drive.moveAllFourToTicks(drive.degreesToTicks(190),-drive.degreesToTicks(190),drive.degreesToTicks(190),-drive.degreesToTicks(190));

        // }
        // else  if (time.get() > 5 && time.get() < 5.1){
        //     //drive.moveAllFourTo(0,0,0,0);
        //     drive.zeromotors();
        //     shootTimer.reset();
        //     shootTimer.start();
        // }
        // else if (time.get() > 5.1 && time.get() < 10){
        //     // if (shootTimer.get() < 1.25){
        //     //     flyhook.moveTopFlyWheel(1);
        //     // }
        //     // else {
        //     //     flyhook.moveTopFlyWheel(1);
        //     //     flyhook.moveBottomFlyWheel(1);
        //     // }
        //     // drive.moveAllFourTo(0,0,0,0);
        //     //sets the error adjustment values to the PID control. 
           
        // }else if (time.get() > 10 && time.get() < 12){
        //     drive.moveFL(0);
        //     drive.moveFR(0);
        //     drive.moveBL(0);
        //     drive.moveBR(0);
        //     if (shootTimer.get() < 1.25){
        //         flyhook.moveTopFlyWheel(1);
        //     }
        //     else {
        //         flyhook.moveTopFlyWheel(1);
        //         flyhook.moveBottomFlyWheel(1);
        //     }

        // }
        // else {
        //     flyhook.moveFly(0);
        // }
        
        drive.printpositions();





    }

    @Override
    public void end(boolean interrupted) {
        time.reset();
    }

    

}
