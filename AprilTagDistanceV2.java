package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import java.util.List;

@TeleOp(name = "April Tag Detector", group = "Concept")
public class AprilTagTest extends LinearOpMode {

    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;

    private static final double TAG_SIZE = 2.0; 
    DcMotor outtakeR = null;

    @Override
    public void runOpMode() {
        initAprilTag();

        telemetry.addData(">>>", "Press Play to start");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            List<AprilTagDetection> currentDetections = aprilTag.getDetections();
            
            telemetry.addData("# AprilTags Detected", currentDetections.size());

            for (AprilTagDetection detection : currentDetections) {
                if (detection.metadata != null) {
                    displayTagInfo(detection);
                } else {
                    telemetry.addLine(String.format("\n==== Unknown TAG ID: %d", detection.id));
                    telemetry.addData("Center", "%.0f, %.0f (pixels)", detection.center.x, detection.center.y);
                }
            }

            telemetry.addLine("\nKey:\nXYZ = X (Right), Y (Forward), Z (Up) dist.");
            telemetry.addLine("PRY = Pitch, Roll & Yaw (XYZ Rotation)");
            telemetry.addLine("RBE = Range, Bearing & Elevation");

            telemetry.update();
            
            sleep(20);
        }
    }

    private void initAprilTag() {
        aprilTag = new AprilTagProcessor.Builder()
            .setDrawAxes(true)
            .setDrawCubeProjection(true)
            .setDrawTagOutline(true)
            .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
            .setTagLibrary(AprilTagGameDatabase.getCurrentGameTagLibrary())
            .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
            .build();

        VisionPortal.Builder builder = new VisionPortal.Builder();
        builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        builder.addProcessor(aprilTag);
        visionPortal = builder.build();
    }

    private void displayTagInfo(AprilTagDetection detection) {
        telemetry.addLine("\n===== ID: " + detection.id + " =====");
        telemetry.addLine("Type: " + detection.metadata.name);
        telemetry.addLine("Range: " + formatDistance(detection.ftcPose.range));
        telemetry.addLine("Bearing: " + formatAngle(detection.ftcPose.bearing));
        telemetry.addLine("Elevation: " + formatAngle(detection.ftcPose.elevation));
        
        telemetry.addLine("XYZ: " + formatPose(detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
        telemetry.addLine("PRY: " + formatPose(detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw));
        
        displayDistanceCalculations(detection);
    }

    private void displayDistanceCalculations(AprilTagDetection detection) {
        double distance = detection.ftcPose.range;
        double power;
        power = distance * 0.1;
        outtakeR.setPower(power);
        
        double horizontalDistance = Math.sqrt(
            detection.ftcPose.x * detection.ftcPose.x + 
            detection.ftcPose.z * detection.ftcPose.z
        );
        
        telemetry.addLine("--- Distance Info ---");
        telemetry.addLine("Straight-line: " + formatDistance(distance));
        telemetry.addLine("Horizontal: " + formatDistance(horizontalDistance));
        telemetry.addLine("Vertical: " + formatDistance(Math.abs(detection.ftcPose.y)));
        
        // Add field position context if available in metadata
        if (detection.metadata != null) {
            telemetry.addLine("Field Position: " + detection.metadata.fieldPosition.toString());
        }
    }

    private String formatDistance(double distance) {
        return String.format("%.1f inches", distance);
    }

    private String formatAngle(double angle) {
        return String.format("%.1f°", angle);
    }

    private String formatPose(double x, double y, double z) {
        return String.format("%.1f, %.1f, %.1f", x, y, z);
    }
}
