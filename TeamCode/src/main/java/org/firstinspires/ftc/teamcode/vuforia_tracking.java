package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.vuforia.HINT;
import com.vuforia.Vuforia;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

/*
 * This OpMode was written for the VuforiaDemo Basics video. This demonstrates basic principles of
 * using VuforiaDemo in FTC.
 */
@Autonomous(name = "vuforia_tracking")
public class vuforia_tracking extends LinearOpMode
{
    private DcMotor motorLeft;
    private DcMotor motorRight;

    private Servo armServo;
    private Servo handServo;

    private static final double ARM_RETRACTED_POSITION = 0.2;
    private static final double ARM_EXTENDED_POSITION = 0.8;

    private static final double HAND_RETRACTED_POSITION = 0.2;
    private static final double HAND_EXTENDED_POSITION = 0.8;

    // Variables to be used for later
    private VuforiaLocalizer vuforiaLocalizer;
    private VuforiaLocalizer.Parameters parameters;
    private VuforiaTrackables visionTargets;
    private VuforiaTrackable target;
    private VuforiaTrackableDefaultListener listener;

    private OpenGLMatrix lastKnownLocation;
    private OpenGLMatrix phoneLocation;
    private static final String VUFORIA_KEY = "ASXyLdL/////AAAAGf4A7KlSBEoCujPF8a5yWOpgGcsajeR0qUV1jBW7eczf/mkmSmLHteLq2FxEW4kDAgCW/RbUJLuNwRFZbpD39wr/VSyRe36k1NhG4sTAaAEpdgEGdLLaUDZB9Tck8cYJnOPS7L1P8bwE/qH++gclClVX7uQtYU1bwxpBris7tgQxlyXlpWIj/S4lCV0qJ8bOiJsb4PN7fhaUqM8upLkJjBFQHC2Ta94axVKBmTJKxB8nNTL+8+ltPIV7Bl5gf/G4z25WE3tl/YG+IjeRO3+odsEXi1GyUkpo+JGi2fqC2Q+cR7dOwh/Cz5B6O+iaiyOgA9WII4t5v9NijwdVFj5aMwDbh3eSCNpMIhBpQlnfcUfh"; // Insert your own key here

    private float robotX = 0;
    private float robotY = 0;
    private float robotAngle = 0;

    boolean hasSeen=false;

    public void runOpMode() throws InterruptedException
    {
        robotX = 0;
        robotY = 0;
        robotAngle = 0;

        setupVuforia();
        motorLeft = hardwareMap.dcMotor.get("left_drive");
        motorRight = hardwareMap.dcMotor.get("right_drive");

        motorRight.setDirection(DcMotor.Direction.REVERSE);

        armServo = hardwareMap.servo.get("armServo");
        handServo = hardwareMap.servo.get("handServo");
        // We don't know where the robot is, so set it to the origin
        // If we don't include this, it would be null, which would cause errors later on
        lastKnownLocation = createMatrix(0, 0, 0, 0, 0, 0);

        waitForStart();

        // Start tracking the targets
        visionTargets.activate();

        while(opModeIsActive())
        {
            // Ask the listener for the latest information on where the robot is
            OpenGLMatrix latestLocation = listener.getUpdatedRobotLocation();

            // The listener will sometimes return null, so we check for that to prevent errors
            if(latestLocation != null)
                lastKnownLocation = latestLocation;

            float[] coordinates = lastKnownLocation.getTranslation().getData();

            robotX = coordinates[0];
            robotY = coordinates[1];
            robotAngle = Orientation.getOrientation(lastKnownLocation, AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle;

            if (robotX < 15 && robotX > 15 && robotY < 1015 && robotY > 985)
            {
                telemetry.addLine("target locked");
            }

            if (!listener.isVisible()||robotY<600)
            {
                motorLeft.setPower(.2);
                motorRight.setPower(.2);
            }

            else{
                if (robotY<800)
                {
                    motorLeft.setPower(.1);
                    motorRight.setPower(.1);
                }
            }

            if (robotY>800)
            {
                motorLeft.setPower(0);
                motorRight.setPower(0);
            }


            // Send information about whether the target is visible, and where the robot is
            telemetry.addData("Tracking " + target.getName(), listener.isVisible());
            telemetry.addData("Last Known Location", formatMatrix(lastKnownLocation));
            // Send telemetry and idle to let hardware catch up
            telemetry.update();
            idle();
        }
    }
    private void setupVuforia()
    {      // Setup parameters to create localizer
        parameters = new VuforiaLocalizer.Parameters(R.id.cameraMonitorViewId); // To remove the camera view from the screen, remove the R.id.cameraMonitorViewId
        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        parameters.useExtendedTracking = false;
        vuforiaLocalizer = ClassFactory.createVuforiaLocalizer(parameters);

        // These are the vision targets that we want to use
        // The string needs to be the name of the appropriate .xml file in the assets folder
        visionTargets = vuforiaLocalizer.loadTrackablesFromAsset("FTC_2016-17");
        Vuforia.setHint(HINT.HINT_MAX_SIMULTANEOUS_IMAGE_TARGETS, 4);

        // Setup the target to be tracked
        target = visionTargets.get(0); // 0 corresponds to the wheels target
        target.setName("Wheels Target");
        target.setLocation(createMatrix(100, 1000, 0, 90, 0, 0));

        // Set phone location on robot
        phoneLocation = createMatrix(0, 0, 0, 90, 0, 0);

        // Setup listener and inform it of phone information
        listener = (VuforiaTrackableDefaultListener) target.getListener();
        listener.setPhoneInformation(phoneLocation, parameters.cameraDirection);
    }

    // Creates a matrix for determining the locations and orientations of objects
    // Units are millimeters for x, y, and z, and degrees for u, v, and w
    private OpenGLMatrix createMatrix(float x, float y, float z, float u, float v, float w)
    {
        return OpenGLMatrix.translation(x, y, z).
                multiplied(Orientation.getRotationMatrix(
                        AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES, u, v, w));
    }

    // Formats a matrix into a readable string
    private String formatMatrix(OpenGLMatrix matrix)
    {
        return matrix.formatAsTransform();
    }
}