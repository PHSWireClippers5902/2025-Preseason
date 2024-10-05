package frc.robot.commands;

import javax.lang.model.util.ElementScanner14;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.FlyWheelAndHook;
import frc.robot.subsystems.MecanumSystem;

public class FlyCommand extends Command{
    //!!IMPORTANT!! THIS IS A CODE SNIPPIT FOR THE 2024 CRESCENDO CHALLENGE!
    Timer lightTimer = new Timer();
    Timer shootTimer = new Timer();
    Timer ampTimer = new Timer();
    XboxController m_xbox;
    double hookpos = 0;
    FlyWheelAndHook wheel;
    MecanumSystem light;
    double GLOBALPOSITION = 0;
    public FlyCommand(XboxController xbox, FlyWheelAndHook fly,MecanumSystem lightSystem){
        m_xbox = xbox;
        lightTimer.reset();
        lightTimer.start();
        shootTimer.reset();
        shootTimer.start();
        ampTimer.reset();
        ampTimer.start();
        wheel = fly;
        
        addRequirements(wheel);
        light = lightSystem;
        
    }
    @Override
    public void execute() {
       
        if (m_xbox.getRightBumper()){
            //wheel.moveFly(m_xbox.getLeftTriggerAxis());
            wheel.moveTopFlyWheel(-0.7);
            wheel.moveBottomFlyWheel(-.4);
            if (Math.round(lightTimer.get()*10) % 2 == 0){
                light.changeColor(0.59);
            }
            else 
            {
                light.changeColor(0.99);
            }
        }
        else if (m_xbox.getLeftTriggerAxis() > 0.1){
            //light.changeColor(1);
            // if (Math.round(lightTimer.get()*4) % 2 == 0){
            //     light.changeColor(0.77);
            // }
            // else 
            // {
            //     light.changeColor(0.99);
            // }
            // wheel.moveFly(m_xbox.getLeftTriggerAxis());
            if (shootTimer.get() < 1.25){
                wheel.moveTopFlyWheel(1);
            }
            else {
                wheel.moveTopFlyWheel(1);
                wheel.moveBottomFlyWheel(1);
            }
            if (Math.round(lightTimer.get()*4) % 2 == 0){
                light.changeColor(0.77);
            }
            else 
            {
                light.changeColor(0.99);
            }

            SmartDashboard.putNumber("LeftTriggerExportSpeed: ",0.3*m_xbox.getLeftTriggerAxis());
            // wheel.moveBottomFlyWheel(0.2);
            // //wheel.moveTopFlyWheel(0.3*m_xbox.getLeftTriggerAxis());
            // wheel.moveTopFlyWheel(.16);
        } 
        // else if (m_xbox.getLeftBumper()){
        //     wheel.moveBottomFlyWheel(0.25);
        //     wheel.moveTopFlyWheel(1);
        
        // }    
        
        
        else if (m_xbox.getRightTriggerAxis() > 0.1)
        {
            if (shootTimer.get() < 1.25){
                wheel.moveTopFlyWheel(1);
            }
            else {
                wheel.moveTopFlyWheel(1);
                wheel.moveBottomFlyWheel(1);
            }
            if (Math.round(lightTimer.get()*4) % 2 == 0){
                light.changeColor(0.77);
            }
            else 
            {
                light.changeColor(0.99);
            }
            // wheel.moveTopFlyWheel(1);
            // wheel.moveBottomFlyWheel(0);
        }


        // else {
        //     wheel.moveFly(0);
        // }
        
        // else if (js.getRawButton(5)){
        //     wheel.moveFly(-0.85);
        // }
        // else if (js.getRawButton(4)){
        //     wheel.moveFly(1);
        // }
        // else if(js.getRawButton(6)){
        //     wheel.moveTopFlyWheel(1);
        //     wheel.moveBottomFlyWheel(0);
        // }
        else {
            shootTimer.reset();
            shootTimer.start();
             if (m_xbox.getXButton()){
            //move the flywheel at AMP speed for x seconds
            if (ampTimer.get() > 1 && ampTimer.get() < 1.5){
                wheel.moveBottomFlyWheel(0.2);
                wheel.moveTopFlyWheel(.16);
            }
            //wheel.moveBottomFlyWheel(0.2);
            //wheel.moveTopFlyWheel(0.3*m_xbox.getLeftTriggerAxis());
            //wheel.moveTopFlyWheel(.16);
            

        }
        else {
            ampTimer.reset();
            ampTimer.start();
            
            wheel.moveFly(0);
        }
        if (wheel.getLimitSwitch()){
            wheel.zeroHook();
            GLOBALPOSITION = wheel.getHookPos();
        }


        }

        if (m_xbox.getYButton()){
            wheel.moveHook(-0.3);
        }
        else if (m_xbox.getAButton()){
            wheel.moveHook(0.3);
        }
        
        else if (m_xbox.getLeftTriggerAxis() > 0.1){
            wheel.encoderLock(0);
        }
        else if (m_xbox.getPOV() == 180){
            if (wheel.getHookPos() > 270){
                wheel.moveHook(0);
            }
            else if (wheel.getHookPos() > 170){
                wheel.moveHook(0.2);
            }
            else {

            
            wheel.encoderLock(270);
            }
            
        }
        else {
            wheel.moveHook(0);
        }
        SmartDashboard.putNumber("HookPos: ",wheel.getHookPos());
        SmartDashboard.putBoolean("HookLimit: ", wheel.getLimitSwitch());
        //System.out.println(wheel.getLimitSwitch());
        if (m_xbox.getPOV() == 270){
            if (wheel.getLimitSwitch()){
                wheel.moveHook(0.01);
            }
            else {
                wheel.moveHook(-0.2);
            }

        }
        // if (m_xbox.getXButton()){
        //     wheel.moveFly(0);
        // }
        //code that would be used if used in comp
        
        // if (m_xbox.getAButton()){
        //     wheel.moveFly(1);
        // }
        // if (m_xbox.getAButton()){
        //     wheel.moveActuatorOn();
        // }
        // else if (m_xbox.getBButton()){
        //     wheel.moveActuatorOff();
        // }
        // else {
        //     //wheel.neutralActuator();
        // }

        
    }

    
}
