package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.robotcore.hardware.IMU;
//148808

@TeleOp
public class Field extends LinearOpMode { 

    DcMotor frontLeft, frontRight, backLeft, backRight;
    DcMotor intake;
    DcMotorEx outtakeL;
    DcMotorEx outtakeR;
    Servo gate;
    
    double maxVel = 2000; 
    double targetVel = 0;
    
    double closed = 0.4;
    double open = 0.6;
    
    public static double kP = 0.0035;
    public static double kI = 0.0001;
    public static double kD = 0.00031;
    public static double kF = 0.0005; 

    double integralSum = 0;
    double lastError = 0;
    public ElapsedTime PIDTimer = new ElapsedTime();
    
    public ElapsedTime shot = new ElapsedTime();
    
    public enum ShooterState {
        IDLE,
        SPINUP,
        READY,
        FEEDING,
        COOLDOWN
    }
    public ShooterState currentState = ShooterState.IDLE;

    @Override
    public void runOpMode() {
        
        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("fl");
        DcMotor backLeftMotor = hardwareMap.dcMotor.get("bl");
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("fr");
        DcMotor backRightMotor = hardwareMap.dcMotor.get("br");
        
        intake = hardwareMap.get(DcMotor.class, "intake");
        outtakeL = hardwareMap.get(DcMotorEx.class, "outtakeL");
        outtakeR = hardwareMap.get(DcMotorEx.class, "outtakeR");
        gate = hardwareMap.get(Servo.class, "servo");

        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        intake.setDirection(DcMotorSimple.Direction.FORWARD);
        
        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        outtakeR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        outtakeL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        outtakeL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        outtakeR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        IMU imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
        imu.initialize(parameters);

        telemetry.addLine("Initialized — ready to start");
        telemetry.update();
        gate.setPosition(closed);

        waitForStart();
        shot.reset();
        PIDTimer.reset();
        if (isStopRequested()) return;

        while (opModeIsActive()) {
            
            double x = gamepad1.left_stick_y;
            double y = gamepad1.left_stick_x;
            double rx = -gamepad1.right_stick_x;

            if (gamepad1.a) {
                imu.resetYaw();
            }

            double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
            double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

            rotX = rotX * 1.1; 

            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
            double frontLeftPower = (rotY - rotX - rx) / denominator;
            double backLeftPower = (rotY + rotX - rx) / denominator;
            double frontRightPower = (rotY + rotX + rx) / denominator;
            double backRightPower = (rotY - rotX + rx) / denominator;

            frontLeftMotor.setPower(frontLeftPower);
            backLeftMotor.setPower(backLeftPower);
            frontRightMotor.setPower(frontRightPower);
            backRightMotor.setPower(backRightPower);
            
            double currentVel = outtakeR.getVelocity();
            double error = targetVel - currentVel;
            double dt = PIDTimer.seconds();
            PIDTimer.reset();
            double p = kP * error;
            integralSum += (error * dt);
            integralSum = Range.clip(integralSum, -1.0, 1.0); 
            double i = kI * integralSum;
            double derivative = (error - lastError) / dt;
            lastError = error;
            double d = kD * derivative;
            double f = kF * targetVel;
            double power = p + i + d + f;
            power = Range.clip(power, -1.0, 1.0);
            outtakeL.setPower(-power); 
            outtakeR.setPower(power); 

            if (gamepad1.left_bumper || gamepad2.left_bumper || gamepad1.left_trigger > 0.1 || gamepad2.left_trigger > 0.1) {
                intake.setPower(-1.0); 
                gate.setPosition(open);
            } else if (gamepad1.right_trigger >0.1 || gamepad2.right_trigger >0.1) {
                gate.setPosition(closed);
                intake.setPower(1.0);
            } else if (gamepad1.right_bumper){
            } else {
                if (currentState != ShooterState.FEEDING) {
                    intake.setPower(0);
                }
                if (currentState != ShooterState.FEEDING) {
                    gate.setPosition(closed);
                }
            }
    
            switch(currentState){
                case IDLE:
                    targetVel = 0.7 * maxVel;
                    gate.setPosition(closed);
                    
                    if(gamepad1.right_bumper || gamepad2.right_bumper){
                        targetVel = 0.7 * maxVel;
                        currentState = ShooterState.SPINUP;
                    }
                    break;
                
                case SPINUP:
                    gate.setPosition(closed);
                    if(!(gamepad1.right_bumper||gamepad2.right_bumper)) {
                        currentState = ShooterState.IDLE;
                    } else if ( velGood(currentVel, targetVel) ) {  
                        currentState = ShooterState.READY;
                    }
                    break;
                
                case READY:
                    gate.setPosition(closed);
                    if (!(gamepad1.right_bumper||gamepad2.right_bumper)) {
                        currentState = ShooterState.IDLE;
                    } else if (!velGood(currentVel, targetVel)){ 
                        currentState = ShooterState.SPINUP;
                    } else {
                        intake.setPower(1.0);
                        shot.reset();
                        currentState = ShooterState.FEEDING;
                        gate.setPosition(open);
                    }
                    break;
                    
                case FEEDING:
                    if (!(gamepad1.right_bumper||gamepad2.right_bumper)) {
                        currentState = ShooterState.IDLE;
                    } else if (shot.seconds() >= 0.23){
                        intake.setPower(0.0);
                        shot.reset();
                        currentState = ShooterState.COOLDOWN;
                        gate.setPosition(closed);
                    }
                    break;
                    
                case COOLDOWN:
                    if(!(gamepad1.right_bumper||gamepad1.right_bumper)){
                        currentState = ShooterState.IDLE;
                    } else if(shot.seconds() > 0.3) { 
                        currentState = ShooterState.SPINUP; 
                    }
                    break;
            }
            
            telemetry.addData("Current State", currentState);
            telemetry.addData("Shot Clock ", shot.seconds());
            telemetry.addLine("---------------------");
            telemetry.addData("Target Vel", targetVel);
            telemetry.addData("Current Vel (Right)", "%.2f", currentVel); 
            telemetry.addData("Error", "%.2f", error);
            telemetry.addData("PID Power", "%.2f", power);
            telemetry.addData("velGood: ", velGood(currentVel, targetVel) ); 
            telemetry.addLine("---------------------");
            telemetry.addData("velocity RIGHT (Raw)", outtakeR.getVelocity());
            telemetry.addData("velocity LEFT (Raw)", outtakeL.getVelocity()); 
            telemetry.addData("gate pos", gate.getPosition());
            telemetry.addData("Intake Power", "%.2f", intake.getPower());
            
            telemetry.update();
        }
    }
    
    public boolean velGood (double currentVelocity, double targetVelocity){
        if (targetVelocity == 0) {
            return Math.abs(currentVelocity) < 50; 
        }
        double minVal = targetVelocity * 0.95;
        double maxVal = targetVelocity * 1.05;
        return (currentVelocity > minVal) && (currentVelocity < maxVal); 
    }
}
