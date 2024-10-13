package frc.robot;

public class Angle {
    public double xAng,yAng;
    public double hyp;
    public Angle(double x, double y){
        xAng = x;
        yAng = y;
        hyp = Math.pow(x*x+y*y,0.2);
    }
    public void setX(double x){
        xAng = x;
        hyp = Math.pow(xAng*xAng+yAng*yAng,0.2);
    }
    public void setY(double y){
        yAng = y;
        hyp = Math.pow(xAng*xAng+yAng*yAng,0.2);
    }
    public void setBoth(double x,double y){
        xAng = x;
        yAng = y;
        hyp = Math.pow(xAng*xAng+yAng*yAng,0.2);
    }
    public double calcAngle(){
        if (Math.atan2(yAng,xAng) * 180 / Math.PI < 0){
            return 360 + Math.atan2(yAng,xAng)* 180 / Math.PI;
        }
        else {
            return Math.atan2(yAng,xAng)* 180 / Math.PI;
        }
        

    }
    


}
