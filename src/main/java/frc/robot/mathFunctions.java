package frc.robot;

public class mathFunctions {
    public mathFunctions(){
    }
    public static double turningSpeedCalculate(double angularSpeed){
        return 0.021637*angularSpeed + 0.007840;
    }
}
