
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

public class Thingy extends LinearOpMode {

    private DcMotor frontleft = null;
    private DcMotor backleft = null;
    private DcMotor frontright = null;
    private DcMotor backright = null;
    DcMotor outtakeL = null;
    DcMotor outtakeR = null;
    DcMotor intake = null;
    ElapsedTime timer = new ElapsedTime();
    double time = timer.seconds();
    
    @Override
    public void runOpMode() {
        
        frontleft = hardwareMap.get(DcMotor.class, "fl");
        frontright = hardwareMap.get(DcMotor.class, "fr");
        backleft = hardwareMap.get(DcMotor.class, "bl");
        backright = hardwareMap.get(DcMotor.class, "br");
        intake = hardwareMap.get(DcMotor.class, "intake");
        outtakeR = hardwareMap.get(DcMotor.class, "outtakeR");
        outtakeL = hardwareMap.get(DcMotor.class, "outtakeL");
        
      
        
        waitForStart();
        
       
                               
        while (opModeIsActive()) {
            
            outtakeR.setPower(0.65);
            outtakeL.setPower(-0.65);
            backward();
            sleep(40);
            shoot();
            sleep(1000);
            shoot();
            break;
        }
        
        }
    
        public void backward(){
            
            frontleft.setPower(0.3);
            frontright.setPower(-0.3);
            backleft.setPower(0.3);
            backright.setPower(-0.3);
            sleep(1500);
            frontleft.setPower(0);
            frontright.setPower(0);
            backleft.setPower(0);
            backright.setPower(0);
        }
        public void right(){
    
            frontleft.setPower(-0.5);
            frontright.setPower(-0.5);
            backleft.setPower(0.5);
            backright.setPower(0.5);
            sleep(300);
            frontleft.setPower(0);
            frontright.setPower(0);
            backleft.setPower(0);
            backright.setPower(0);
        }
        public void shoot(){
        
            intake.setPower(1);
            sleep(200);
            intake.setPower(0);
        
        }
    }
