package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


@Autonomous(name="bucket auto", group="2024 Auto")
//@Disabled
public class bucketAuto extends OpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime(ElapsedTime.Resolution.SECONDS);

    private DcMotor leftFrontMotor;
    private DcMotor leftBackMotor;
    private DcMotor rightFrontMotor;
    private DcMotor rightBackMotor;
    private IMU imu;






    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");

        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
        leftFrontMotor  = hardwareMap.get(DcMotor.class, "left_front_drive");
        leftBackMotor  = hardwareMap.get(DcMotor.class, "left_back_drive");
        rightFrontMotor = hardwareMap.get(DcMotor.class, "right_front_drive");
        rightBackMotor = hardwareMap.get(DcMotor.class, "right_back_drive");

        leftFrontMotor.setDirection(DcMotor.Direction.REVERSE);
        leftBackMotor.setDirection(DcMotor.Direction.REVERSE);
        rightFrontMotor.setDirection(DcMotor.Direction.FORWARD);
        rightBackMotor.setDirection(DcMotor.Direction.FORWARD);




        // Retrieve the IMU from the hardware map
        IMU imu = hardwareMap.get(IMU.class, "imu");
        //FIXME Adjust the orientation parameters to match your robot
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.RIGHT));
        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
        imu.initialize(parameters);


        // Wait for the game to start (driver presses PLAY)
        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    /*
     * Code to run ONCE when the driver hits START
     */
    @Override
    public void start() {
        runtime.reset();
    }

    /*
     * Code to run REPEATEDLY after the driver hits START but before they hit STOP
     */
    @Override
    public void loop() {


// score samples
        if (runtime.time() < 3) { // drive forward
            driveForward(0.2);
        }

        if (runtime.time() < 6) { // strafes past the samples
            driveLeft(0.2);
        }

        if (runtime.time() < 8) { // drives backwards up against the wall
            driveBack(0.2);
        }

        if (runtime.time() < 13) { // pushes samples into scoring area
            driveRight(0.2);
        }

// realign robot
        if (runtime.time() < 15) { // goes left to get off right wall
            driveLeft(0.1);
        }

        if (runtime.time() < 18) { // push up against wall to straighten robot
            driveBack(0.15);
        }

        if (runtime.time() < 20) { // drive forward to get off wall
            driveForward(0.1);
        }

// park in observation zone
        if (runtime.time() < 25) { // goes forward to line up with observation zone
            driveForward(0.2);
        }

        if (runtime.time() < 27) { // drive into observation zone
            driveRight(0.2);
        }
    }



    public void driveForward(double speed) {
        leftFrontMotor.setPower(speed);
        leftBackMotor.setPower(speed);
        rightFrontMotor.setPower(speed);
        rightBackMotor.setPower(speed);
    }
    public void driveBack(double speed) {
        leftFrontMotor.setPower(-speed);
        leftBackMotor.setPower(-speed);
        rightFrontMotor.setPower(-speed);
        rightBackMotor.setPower(-speed);
    }
    public void driveLeft(double speed) {
        leftFrontMotor.setPower(-speed);
        leftBackMotor.setPower(speed);
        rightFrontMotor.setPower(speed);
        rightBackMotor.setPower(-speed);
    }
    public void driveRight(double speed) {
        leftFrontMotor.setPower(speed);
        leftBackMotor.setPower(-speed);
        rightFrontMotor.setPower(-speed);
        rightBackMotor.setPower(speed);
    }
    public void turnLeft(double speed) {
        leftFrontMotor.setPower(-speed);
        leftBackMotor.setPower(-speed);
        rightFrontMotor.setPower(speed);
        rightBackMotor.setPower(speed);
    }
    public void turnRight(double speed) {
        leftFrontMotor.setPower(speed);
        leftBackMotor.setPower(speed);
        rightFrontMotor.setPower(-speed);
        rightBackMotor.setPower(-speed);
    }




}
