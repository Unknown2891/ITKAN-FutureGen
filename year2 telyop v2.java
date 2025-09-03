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

public class LuqmanTeleop extends LinearOpMode {

    DcMotor fr = null;
    DcMotor fl = null;
    DcMotor br = null;
    DcMotor bl = null;
    double leftstick = 0;
    double rightstick = 0;
    
    DcMotor armRight = null;
    DcMotor armLeft = null;

    private DcMotor slider;
    int sliderTarget = -100;
    
    Servo claw = null;

    @Override
    public void runOpMode() {

        fr = hardwareMap.get(DcMotor.class, "fr");
        fl = hardwareMap.get(DcMotor.class, "fl");
        br = hardwareMap.get(DcMotor.class, "br");
        bl = hardwareMap.get(DcMotor.class, "bl");
        slider = hardwareMap.get(DcMotor.class, "slider");
        armRight = hardwareMap.get(DcMotor.class, "rightArm");
        armLeft = hardwareMap.get(DcMotor.class, "leftArm");
        claw = hardwareMap.get(Servo.class, "claw");

        fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slider.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        slider.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("Status", "Running");
            leftstick = gamepad1.left_stick_y;
            rightstick = gamepad1.right_stick_y;

            fr.setPower(-rightstick);
            br.setPower(-rightstick);
            fl.setPower(leftstick);
            bl.setPower(leftstick);
            
             if (gamepad1.left_bumper || gamepad2.left_bumper) {
                slider.setTargetPosition(slider.getCurrentPosition() + 30);
                slider.setPower(0.8);
                slider.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            }
           
           
            if (gamepad1.right_bumper || gamepad2.right_bumper) {
                slider.setTargetPosition(slider.getCurrentPosition() - 30);
                slider.setPower(0.8);
                slider.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            }
            
            if (gamepad1.x || gamepad2.x) {
                slider.setTargetPosition(10);
                slider.setPower(0.7);
                slider.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            }
           
           
            if (gamepad1.a || gamepad2.a) {
                slider.setTargetPosition(-3700);
                slider.setPower(0.7);
                slider.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            }
            
             if (gamepad1.b || gamepad2.b) {
                 
                claw.setPosition(0.7);
                
            }
           
            if (gamepad1.y || gamepad2.y) {
                
                claw.setPosition(0);

            }
            
            if (gamepad1.left_trigger > 0.05 || gamepad2.left_trigger > 0.05) {
                armLeft.setTargetPosition(armLeft.getCurrentPosition() + 20);
                armRight.setTargetPosition(armRight.getCurrentPosition() - 20);
                armRight.setPower(0.5);
                armLeft.setPower(0.5);
                armLeft.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                armRight.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                telemetry.addData("Left trigger: ", gamepad1.left_trigger);
            }
           
            if (gamepad1.right_trigger > 0.5 || gamepad2.right_trigger > 0.5) {
                armLeft.setTargetPosition(armLeft.getCurrentPosition() - 20);
                armRight.setTargetPosition(armRight.getCurrentPosition() + 20);
                armRight.setPower(0.5);
                armLeft.setPower(0.5);
                armLeft.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                armRight.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                telemetry.addData("Right trigger:", gamepad1.right_trigger);
            }

            telemetry.addData("Left Stick:", leftstick);
            telemetry.addData("Right Stick:", rightstick);
            telemetry.addData("Status", "Running");
            telemetry.addData("slider position", slider.getCurrentPosition());
            telemetry.addData("left arm position", armLeft.getCurrentPosition());
            telemetry.addData("right arm position", armRight.getCurrentPosition());
            telemetry.update();

        }
    }
}
