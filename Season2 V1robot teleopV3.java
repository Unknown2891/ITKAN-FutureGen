package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp
public class RobotTeleop extends LinearOpMode {

    DcMotor frontLeft, frontRight, backLeft, backRight;
    DcMotor intake;
    DcMotor outtakeL;
    DcMotor outtakeR;

    @Override
    public void runOpMode() {
        
        frontLeft = hardwareMap.get(DcMotor.class, "fl");
        frontRight = hardwareMap.get(DcMotor.class, "fr");
        backLeft = hardwareMap.get(DcMotor.class, "bl");
        backRight = hardwareMap.get(DcMotor.class, "br");
        intake = hardwareMap.get(DcMotor.class, "intake");
        outtakeL = hardwareMap.get(DcMotor.class, "outtakeL");
        outtakeR = hardwareMap.get(DcMotor.class, "outtakeR");

        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        backRight.setDirection(DcMotorSimple.Direction.FORWARD);
        intake.setDirection(DcMotorSimple.Direction.FORWARD);
        
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        outtakeR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        outtakeL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addLine("Initialized — ready to start");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            
            double drive = -gamepad1.left_stick_y;  
            double strafe = -gamepad1.left_stick_x; 
            double turn = -gamepad1.right_stick_x;  

            double flPower = drive + strafe + turn;
            double frPower = drive - strafe - turn;
            double blPower = drive - strafe + turn;
            double brPower = drive + strafe - turn;

            double max = Math.max(1.0, Math.abs(flPower));
            max = Math.max(max, Math.abs(frPower));
            max = Math.max(max, Math.abs(blPower));
            max = Math.max(max, Math.abs(brPower));

            frontLeft.setPower(flPower / max);
            frontRight.setPower(frPower / max);
            backLeft.setPower(blPower / max);
            backRight.setPower(brPower / max);

            if (gamepad1.left_bumper) {
                telemetry.addLine("left_bumper");

                intake.setPower(-1.0); 
            } else if (gamepad1.right_bumper) {
                telemetry.addLine("right_bumper");

                intake.setPower(1.0);
            } else {
                intake.setPower(0.0); 
            }
            
            if(gamepad1.x){
                outtakeL.setPower(-0.65);
                outtakeR.setPower(0.65);
            }else if(gamepad1.y){
                outtakeL.setPower(0);
                outtakeR.setPower(0);
            }else if(gamepad1.b){
                outtakeL.setPower(0.5);
                outtakeR.setPower(-0.5);
            }
            telemetry.addData("Drive", "%.2f", drive);
            telemetry.addData("Strafe", "%.2f", strafe);
            telemetry.addData("Turn", "%.2f", turn);
            telemetry.addData("Intake Power", "%.2f", intake.getPower());
            telemetry.update();
        }
    }
}
