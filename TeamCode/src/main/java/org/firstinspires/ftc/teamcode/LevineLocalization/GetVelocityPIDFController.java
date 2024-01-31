package org.firstinspires.ftc.teamcode.LevineLocalization;

import com.qualcomm.robotcore.hardware.PIDFCoefficients;

public class GetVelocityPIDFController {
    private double kp; // Proportional gain
    private double ki; // Integral gain
    private double kd; // Derivative gain
    private double kf; // Feedforward gain
    private double setpoint;
    private double integral;
    private double previousError;

    public GetVelocityPIDFController(PIDFCoefficients PIDFc, double velocity) {
        this.kp = PIDFc.p;
        this.ki = PIDFc.i;
        this.kd = PIDFc.d;
        this.kf = PIDFc.f;
        this.setpoint = velocity;
        this.integral = 0;
        this.previousError = 0;
    }

    public double calculateMotorPow(double currVel) {
        double error = setpoint - currVel;

        double proportionalTerm = kp * error;

        integral += error;
        double integralTerm = ki * integral;

        double derivative = error - previousError;
        double derivativeTerm = kd * derivative;

        double feedforwardTerm = kf * setpoint;

        double output = proportionalTerm + integralTerm + derivativeTerm + feedforwardTerm;

        previousError = error;

        return output;
    }
}