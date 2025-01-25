
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.Servo;


@TeleOp

public class Intake extends LinearOpMode {
    
    // intake
    Servo blocker = null;
    DcMotor intake = null;
    
    // slider
    DcMotor rslider = null;
    DcMotor lslider = null;

    // arm
    DcMotor arm = null;
    final double minArmPos = 0;
    final double maxArmPos = 80;
    double armSpeed = 1;
    int armTarget = 0;
    int rsliderTarget = -100;
    int lsliderTarget = 100;

    // pid
    static double kp = 0.5;
    static double ki = 0;
    static double kd = 0.001;
    double previousError = 0;
    double integralSum = 0;
    ElapsedTime time = new ElapsedTime();

    @Override
    public void runOpMode() {
        
        blocker= hardwareMap.get(Servo.class,"blocker");
        //intake = hardwareMap.get(DcMotor.class,"intake");
        arm = hardwareMap.get(DcMotor.class,"arm");
        rslider = hardwareMap.get(DcMotor.class,"rslider");
        lslider = hardwareMap.get(DcMotor.class,"lslider");

        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rslider.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lslider.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rslider.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lslider.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        waitForStart();

        while (opModeIsActive()) {
            
            blocker.setPosition(0.85);
            
            if(gamepad1.y){
                //intake.setPower(1);
            }
            else if(gamepad1.b){
                //intake.setPower(0);
            }
            else if(gamepad1.a){
                //intake.setPower(-1);
            }
            
            if(gamepad1.dpad_right) {
                rsliderTarget = -1700;
                lsliderTarget = 1700;
            }
            
            if(gamepad1.dpad_left) {
                rsliderTarget = -100;
                lsliderTarget = 100;
            }
            
            //moveArm();
            if (gamepad1.dpad_up) {
                armTarget = -2000; 
            }
            if (gamepad1.dpad_down) {
                armTarget = -200; 
            }
            
            
            arm.setTargetPosition(armTarget);
            arm.setPower(1);
            arm.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            
            rslider.setTargetPosition(rsliderTarget);
            rslider.setPower(1);
            rslider.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            
            lslider.setTargetPosition(lsliderTarget);
            lslider.setPower(1);
            lslider.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            
            telemetry.addData("arm position", arm.getCurrentPosition());
            telemetry.addData("rslider position", rslider.getCurrentPosition());
            telemetry.addData("lslider position", lslider.getCurrentPosition());
            telemetry.update();
        }

    }

    public void moveArm() {
        int currentPos = arm.getCurrentPosition();
        
        if (gamepad1.dpad_up) {
            //armTarget = armTarget - armSpeed; 
        }
        if (gamepad1.dpad_down) {
            //armTarget = armTarget + armSpeed;
        }

        if (armTarget > maxArmPos){
            //armTarget = maxArmPos;
        }
        if (armTarget < minArmPos){
            //armTarget = minArmPos;
        }

        telemetry.addData("arm current", currentPos);
        telemetry.addData("arm target", armTarget);
        telemetry.update();
            
        arm.setPower(pidController(armTarget, currentPos));
    }

    public double pidController(double target, double current) {
        double proportionError = target - current;

        integralSum += proportionError * time.seconds();

        double derivativeError = (proportionError - previousError) / time.time();
        time.reset();
        previousError = proportionError;

        integralSum *= ki;
        proportionError *= kp;
        derivativeError *= kd;

        double output = proportionError + integralSum + derivativeError;

        return output;
    }
  
    public double anglewrap(double degrees) {
        while (degrees > 180) {
            degrees -= 360;
        }

        while (degrees < -180) {
            degrees += 360;
        }

        return degrees;
    }
}
