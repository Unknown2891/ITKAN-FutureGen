
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


@TeleOp

public class Telemetery extends LinearOpMode {

    private DcMotor frontleft = null;
    private DcMotor backleft = null;
    private DcMotor frontright = null;
    private DcMotor backright = null;
    private DcMotor arm = null;
    private DcMotor rslider = null;
    private DcMotor lslider = null;
    double armSpeed = 1;
    int armTarget = 0;
    int rsliderTarget = -100;
    int lsliderTarget = 100;
    boolean printMessages = true;

    @Override
    public void runOpMode() {
        
        frontleft = hardwareMap.get(DcMotor.class, "fl");
        frontright = hardwareMap.get(DcMotor.class, "fr");
        backleft = hardwareMap.get(DcMotor.class, "bl");
        backright = hardwareMap.get(DcMotor.class, "br");
        arm = hardwareMap.get(DcMotor.class,"arm");
        rslider = hardwareMap.get(DcMotor.class,"rslider");
        lslider = hardwareMap.get(DcMotor.class,"lslider");

        
        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("Status", "Running");
            telemetry.addData("frpower", frontright);
            telemetry.addData("flpower", frontleft);
            telemetry.addData("blpower", backleft);
            telemetry.addData("brpower", backright);
            telemetry.addData("arm position", arm.getCurrentPosition());
            telemetry.addData("rslider position", rslider.getCurrentPosition());
            telemetry.addData("lslider position", lslider.getCurrentPosition());
            telemetry.update();

        }
    }
}
