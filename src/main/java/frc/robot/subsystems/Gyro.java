package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.I2C;

public class Gyro extends SubsystemBase{
    public AHRS m_gyro;
    public Gyro(){
        
        m_gyro = new AHRS(I2C.Port.kMXP);
    }
    public Rotation2d getAng(){
        return m_gyro.getRotation2d();     
    }
    public void reset(){
        m_gyro.reset();
    }

}
