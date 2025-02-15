package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Autonomous(name = "AutonomousForward")
public class AAuton extends LinearOpMode {

    private DcMotor fl, fr, br, bl, arm, lslider, rslider, intake;
    private Servo blocker;

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize hardware
        fl = hardwareMap.get(DcMotor.class, "fl");
        fr = hardwareMap.get(DcMotor.class, "fr");
        bl = hardwareMap.get(DcMotor.class, "bl");
        br = hardwareMap.get(DcMotor.class, "br");
        arm = hardwareMap.get(DcMotor.class, "arm");
        lslider = hardwareMap.get(DcMotor.class, "lslider");
        rslider = hardwareMap.get(DcMotor.class, "rslider");
        intake = hardwareMap.get(DcMotor.class, "intake");
        blocker = hardwareMap.get(Servo.class, "blocker");

        // Set motor directions
        fl.setDirection(DcMotorSimple.Direction.FORWARD);
        bl.setDirection(DcMotorSimple.Direction.FORWARD);
        fr.setDirection(DcMotorSimple.Direction.REVERSE);
        br.setDirection(DcMotorSimple.Direction.REVERSE);
        lslider.setDirection(DcMotorSimple.Direction.REVERSE);
        rslider.setDirection(DcMotorSimple.Direction.REVERSE);
        arm.setDirection(DcMotorSimple.Direction.REVERSE);
        intake.setDirection(DcMotorSimple.Direction.FORWARD);

        // Set zero power behavior
        fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lslider.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rslider.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart();

        if (opModeIsActive()) {
            // Strafe right for 1 second
          //  setStrafePower(0.5);
         //   sleep(500);
          //  stopMotors();

            // Move forward slightly
            setDrivePower(1);
            sleep(180);
            stopMotors();
            sleep(1000);

            // Raise the arm
            arm.setPower(1);
            sleep(480);
            arm.setPower(0);
            holdPosition(arm);
            sleep(1000);
//
            // Extend sliders
            lslider.setPower(-1);
            rslider.setPower(1);
            sleep(2000);
            holdPosition(arm);
            holdPosition(lslider);
            holdPosition(rslider);
//
            lslider.setPower(0);
            rslider.setPower(0);
//
            // Activate intake
            intake.setPower(0.35);
            sleep(1000);
            intake.setPower(0);
            
            setDrivePower(-0.3);
            sleep(300);
            stopMotors();
            sleep(1000);
            
            // arm.setPower(-0.2);
            // sleep(100);
            // arm.setPower(0);
            // holdPosition(arm);
            // sleep(1000);
        
            // setDrivePower(-1);
            // sleep(100);
            // stopMotors();
            // sleep(1000);
//
            // Retract sliders
            lslider.setPower(1);
            rslider.setPower(-1);
            sleep(2000);
//
            //// Turn right for 400ms
            //setTurnPower(1);
            //sleep(300);
            //stopMotors();
            ////bring arm down
            //  arm.setPower(-1);
            //sleep(480);
            //arm.setPower(0);
            //holdPosition(arm);
            //sleep(1000);
            ////extend sliders
            // lslider.setPower(-1);
            //rslider.setPower(1);
            //sleep(1300);
            //holdPosition(arm);
            //holdPosition(lslider);
            //holdPosition(rslider);
            //
            // intake.setPower(-0.75);
            //sleep(1000);
            //intake.setPower(0);
            //
            // lslider.setPower(1);
            //  intake.setPower(-0.75);
            //rslider.setPower(-1);
            //sleep(2000);
            //setTurnPower(-1);
            //sleep(170);
            //stopMotors();
        }
    }

    private void setDrivePower(double power) {
        fl.setPower(power);
        fr.setPower(power);
        bl.setPower(power);
        br.setPower(power);
    }

    private void setStrafePower(double power) {
        fl.setPower(power);
        fr.setPower(-power);
        bl.setPower(-power);
        br.setPower(power);
    }

    private void setTurnPower(double power) {
        fl.setPower(power);
        fr.setPower(-power);
        bl.setPower(power);
        br.setPower(-power);
    }

    private void stopMotors() {
        setDrivePower(0);
    }

    private void holdPosition(DcMotor motor) {
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor.setPower(0);
    }
}
