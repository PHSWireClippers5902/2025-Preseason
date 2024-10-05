package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.proto.Controller;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class SwerveSystem extends SubsystemBase {
    
    
    public WPI_TalonSRX controlOne;
    public double controlOnePosition;
    public CANSparkMax powerOne;
    public SparkPIDController powerOneController;// = hook.getPIDController();
    public RelativeEncoder powerOneRelativeEncoder;// = hook.getEncoder();

    public SwerveSystem(){
        createMotorObjects();
        normalConfigurations();
    }

    public void createMotorObjects(){
        controlOne = new WPI_TalonSRX(6);
        powerOne = new CANSparkMax(7,MotorType.kBrushless);
        controlOnePosition = controlOne.getSelectedSensorPosition(0);
    }
    //theory
    public void moveControlOne(double theta){
        double pos = theta / 4096 * 360;
        controlOne.set(ControlMode.Position,pos);
    }
    public void movePowerOne(double speed){
        powerOne.set(speed);
    }
    
    public void normalConfigurations(){
        controlOne.configFactoryDefault();
        controlOne.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative,0,1);
        controlOne.setSelectedSensorPosition(0,0,1);
    }
    
    

    
    public void zeromotors(){
        controlOne.setSelectedSensorPosition(0,0,0);
    }

    public double getControlOnePosition(){
        return controlOne.getSelectedSensorPosition();
    }
    public double getControlOneRotations(){
        return controlOne.getSelectedSensorPosition() /4096;
    }

    
}
