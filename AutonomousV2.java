/*
Copyright 2025 

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and
associated documentation files (the "Software"), to deal in the Software without restriction,
including without limitation the rights to use, copy, modify, merge, publish, distribute,
sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial
portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT
NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.IMU;


@Autonomous

public class AutonomousLeft extends LinearOpMode {

    private DcMotor frontleft = null;
    private DcMotor backleft = null;
    private DcMotor frontright = null;
    private DcMotor backright = null;
    
    // intake
    Servo blocker = null;
    DcMotor intake = null;
    
    // slider
    DcMotor rslider = null;
    DcMotor lslider = null;

    // arm
    DcMotor arm = null;

    double drive, turn, strafe;
    double flpower, frpower, blpower, brpower;
    IMU imu;

    @Override
    public void runOpMode() {
        
        imu = hardwareMap.get(IMU.class, "imu");
        
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);
        
        imu.initialize(new IMU.Parameters(orientationOnRobot));
        
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        
        frontleft = hardwareMap.get(DcMotor.class, "fl");
        frontright = hardwareMap.get(DcMotor.class, "fr");
        backleft = hardwareMap.get(DcMotor.class, "bl");
        backright = hardwareMap.get(DcMotor.class, "br");
        intake = hardwareMap.get(DcMotor.class,"intake");
        arm = hardwareMap.get(DcMotor.class,"arm");
        rslider = hardwareMap.get(DcMotor.class,"rslider");
        lslider = hardwareMap.get(DcMotor.class,"lslider");
        
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rslider.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lslider.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rslider.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lslider.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        
        frontleft.setDirection(DcMotor.Direction.FORWARD);
        frontright.setDirection(DcMotor.Direction.REVERSE);
        backleft.setDirection(DcMotor.Direction.FORWARD);
        backright.setDirection(DcMotor.Direction.REVERSE);
        
        waitForStart();

        drive = 0.3;
        strafe = 0;
        turn = 0;
        
        frpower = drive - turn - strafe;
        brpower = drive - turn + strafe;
        flpower = drive + turn + strafe;
        blpower = drive + turn - strafe;
        
        frontright.setPower(frpower);
        backright.setPower(brpower);
        frontleft.setPower(flpower);
        backleft.setPower(blpower);
        sleep(400);
        frontright.setPower(0);
        backright.setPower(0);
        frontleft.setPower(0);
        backleft.setPower(0);
        
        arm.setTargetPosition(-2000);
        arm.setPower(1);
        arm.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        sleep(1000);
    
        rslider.setTargetPosition(-2240);
        rslider.setPower(0.5);
        rslider.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        
        lslider.setTargetPosition(2240);
        lslider.setPower(0.5);
        lslider.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        sleep(3000);
        
        intake.setPower(0.2);
        sleep(500);
        intake.setPower(0);
        
        rslider.setTargetPosition(-100);
        rslider.setPower(0.5);
        rslider.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        
        lslider.setTargetPosition(100);
        lslider.setPower(0.5);
        lslider.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

        while (opModeIsActive()) {
            
           

            telemetry.addData("Status", "Running");
            telemetry.addData("Yaw (Z)", "%.2f Deg. (Heading)", orientation.getYaw(AngleUnit.DEGREES));
            telemetry.addData("Pitch (X)", "%.2f Deg.", orientation.getPitch(AngleUnit.DEGREES));
            telemetry.addData("Roll (Y)", "%.2f Deg.\n", orientation.getRoll(AngleUnit.DEGREES));
            telemetry.addData("rslider position", rslider.getCurrentPosition());
            telemetry.addData("lslider position", lslider.getCurrentPosition());
            telemetry.update();
        }
    }
}
