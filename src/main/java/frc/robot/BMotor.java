package frc.robot;

//import edu.wpi.first.wpilibj.motorcontrol.Victor;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;



public class BMotor extends VictorSPX {

    private double currentSpeed;

    /**
     * Simple interpolation function.
     *
     * @param from    current value
     * @param to      target value
     * @param factor  easing factor
     *
     * @return        new value
     */
    private double ease(double from, double to, double factor) {
        return from * (1 - factor) + to * factor;
    }

    public BMotor(int deviceNumber) {
        super(deviceNumber);
    }

    /**
     * Sets the appropriate output on the motor.
     * @param speed Double between -1.0 and 1.0, with 0.0 as stopped.
     */
    public void setSpeed(double speed) {
        set(ControlMode.PercentOutput, speed);
        currentSpeed = speed;
    }

    /**
     * Eases into a target motor speed using an interpolation formula.
     *
     * @param targetSpeed  target speed
     * @param factor       interpolation factor (0-1; higher is faster)
     */
    public void easeSpeed(double targetSpeed, double factor) {
        setSpeed(ease(currentSpeed, targetSpeed, factor));
    }
}