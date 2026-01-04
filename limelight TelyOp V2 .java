package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@TeleOp
public class Limelight extends OpMode {
    
    //Motors and other things declaration;
    private Limelight3A limelight;
    private IMU imu;
    private double distance;
    DcMotor fr, fl, br, bl, transfer, intake;
    DcMotorEx outtakeR, outtakeL;
    Servo gate;
    double open = 0.3;
    double closed = 0.7;
    
    //align;
    boolean autoAligning = false;

    @Override
    public void init(){
        
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        fr = hardwareMap.get(DcMotor.class,"fr");
        fl = hardwareMap.get(DcMotor.class,"fl");
        br = hardwareMap.get(DcMotor.class,"br");
        bl = hardwareMap.get(DcMotor.class,"bl");
        
        outtakeR = hardwareMap.get(DcMotorEx.class,"outtakeR");
        outtakeL = hardwareMap.get(DcMotorEx.class,"outtakeL");
        transfer = hardwareMap.get(DcMotor.class,"transfer");
        intake = hardwareMap.get(DcMotor.class,"intake");
        gate = hardwareMap.get(Servo.class,"gate");
        
        limelight.pipelineSwitch(0);
        
        imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot revHubOrientationOnRobot = new RevHubOrientationOnRobot(
            RevHubOrientationOnRobot.LogoFacingDirection.UP,
            RevHubOrientationOnRobot.UsbFacingDirection.FORWARD);
        imu.initialize(new IMU.Parameters(revHubOrientationOnRobot));
        
        fr.setDirection(DcMotor.Direction.REVERSE);
        br.setDirection(DcMotor.Direction.REVERSE);
        fl.setDirection(DcMotor.Direction.FORWARD);
        bl.setDirection(DcMotor.Direction.FORWARD);
        
        outtakeR.setDirection(DcMotor.Direction.REVERSE);
        outtakeL.setDirection(DcMotor.Direction.FORWARD);
        
        fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        
        transfer.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        outtakeR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        outtakeL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

    }

    @Override
    public void start() {
        limelight.start();
   
    }

    @Override
    public void loop() {
    
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        limelight.updateRobotOrientation(orientation.getYaw(AngleUnit.DEGREES));
        LLResult llResult = limelight.getLatestResult();
        double drive = -gamepad1.left_stick_y;  
        double strafe = gamepad1.left_stick_x; 
        double turn = gamepad1.right_stick_x;  
            
            if(gamepad1.right_bumper || gamepad2.right_bumper){
                intake.setPower(1);
                transfer.setPower(1);
            }
            else if(gamepad1.left_bumper || gamepad2.left_bumper){
                intake.setPower(-1);
                transfer.setPower(-1);                                      
            }else{
                intake.setPower(0);
                transfer.setPower(0);
            }
            
        boolean shoot = false;
        
        if (gamepad1.a) {
    
            int target = calculateVelocity(llResult.getTy());
            shoot = true;
            boolean aligned = false;
            // autoAligning
            // transfer.setPower(1);
            // intake.setPower(1);
            if (llResult != null && llResult.isValid()) {
                Pose3D botPose = llResult.getBotpose_MT2();
                distance = getDistaceFromTag(llResult.getTa());
                double tx = llResult.getTx();
                turn = -calculatePID(tx);
                
                
                telemetry.addData("Status", "AUTO-ALIGNING");
                telemetry.addData("Tx", tx);
            
                
                if (Math.abs(tx) < 0.5) {
                    telemetry.addData("align status", "ALIGNED = TRUE");
                    aligned = true;
                    turn = 0;
                }
            }
            double currentVel = outtakeR.getVelocity();
            
            if(target < currentVel){
                telemetry.addData("target", "target < currentVel");
                outtakeR.setPower(0);
                outtakeL.setPower(0);
            }else{
                telemetry.addData("target", "target >= currentVel");
                outtakeR.setPower(1);
                outtakeL.setPower(1);
            } 
            if(Math.abs(target - currentVel) < 50){
                gate.setPosition(open);
              telemetry.addData("gate", "open gate");
            }
           
        }
           
            double flPower = drive + strafe + turn;
            double frPower = drive - strafe - turn;
            double blPower = drive - strafe + turn;
            double brPower = drive + strafe - turn;
            telemetry.addData("turn power", turn);
            fl.setPower(flPower);
            fr.setPower(frPower);
            bl.setPower(blPower);
            br.setPower(brPower);
            
            telemetry.addData("distance", distance);
        
            telemetry.update();
    }
    
    private double calculatePID(double tx) {
        
        double kp = 0.02;
        double kf = 0.1;
        double proportional = tx * kp;
        double feedForward = kf;
        
        if(tx > 0){
            feedForward = -kf;
        }
       
        double output = proportional - feedForward;
        return -output;
    }
    
    private void setMotorPowers(double flPower, double frPower, double blPower, double brPower) {
        fl.setPower(flPower);
        fr.setPower(frPower);
        bl.setPower(blPower);
        br.setPower(brPower);
    }
    
    public double getDistaceFromTag(double ta){
        double scale = 26500; //32000
        double distance = Math.sqrt((scale / ta));
        return distance;
    }
    public int calculateVelocity(double ty){
        if(distance <= 100){
            return 1500;
        }
        if(distance <= 150){
            return 1600;
        }
        if(distance <= 200){
            return 1700;
        }
        if(distance <= 300){
            return 2000;
        }
        return 0;
    }
}
