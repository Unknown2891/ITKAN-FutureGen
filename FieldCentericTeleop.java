package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.Servo;


@TeleOp(name = "FieldCentericTeleOp", group = "LinearOpMode")

public class FieldCentericTeleOp extends LinearOpMode {
    private DcMotor frontleft = null;
    private DcMotor backleft = null;
    private DcMotor frontright = null;
    private DcMotor backright = null;
    IMU imu;
   
    @Override
    public void runOpMode() {

    imu = hardwareMap.get(IMU.class, "imu");
    IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
    RevHubOrientationOnRobot.LogoFacingDirection.UP,
    RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
    imu.initialize(parameters);

    frontleft = hardwareMap.get(DcMotor.class, "fl");
    frontright = hardwareMap.get(DcMotor.class, "fr");
    backleft = hardwareMap.get(DcMotor.class, "bl");
    backright = hardwareMap.get(DcMotor.class, "br");
       
    frontleft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    frontright.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    backleft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    backright.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
       
    frontleft.setDirection(DcMotor.Direction.REVERSE);
    frontright.setDirection(DcMotor.Direction.FORWARD);
    backleft.setDirection(DcMotor.Direction.REVERSE);
    backright.setDirection(DcMotor.Direction.FORWARD);

    telemetry.addData("Status", "Initialized");
    telemetry.update();

    double drive, turn, strafe;
    double flpower, frpower, blpower, brpower;
   
    frpower = 0;
    flpower = 0;
    brpower = 0;
    blpower = 0;

    waitForStart();

    while (opModeIsActive()) {
           
        double y = -gamepad1.left_stick_x;
        double x = gamepad1.left_stick_y;
        double rx = gamepad1.right_stick_x;

        if (gamepad1.options) {
            imu.resetYaw();
        }

        double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
        double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);
           
        rotX = rotX * 1.1;

        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
        double frontLeftPower = (rotX - rotY - rx) / denominator;
        double backLeftPower = (rotX + rotY - rx) / denominator;
        double frontRightPower = (rotX + rotY + rx) / denominator;
        double backRightPower = (rotX - rotY + rx) / denominator;
 
        //double maxPower=Math.max(+-Math.abs(frpower),Math.max(Math.abs(brpower),Math.max(Math.abs(flpower),Math.abs(blpower))));
           
        //if(maxPower > 1){
            //frpower /= maxPower;
            //brpower /= maxPower;
            //flpower /= maxPower;
            //blpower /= maxPower;
        //}
           
        frontright.setPower(frontLeftPower);
        backright.setPower(backLeftPower);
        frontleft.setPower(frontRightPower);
        backleft.setPower(backRightPower);
         
        telemetry.addData("Status", "Running");
        telemetry.addData("frpower", frpower);
        telemetry.addData("flpower", flpower);
        telemetry.addData("blpower", blpower);
        telemetry.addData("brpower", brpower);
        telemetry.update();

        }
    }
}
