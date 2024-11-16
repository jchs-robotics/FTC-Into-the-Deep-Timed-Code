/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;


@TeleOp(name="timed test", group="2024 Teleop")
//@Disabled
public class timedTest extends OpMode
{
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFrontMotor;
    private DcMotor leftBackMotor;
    private DcMotor rightFrontMotor;
    private DcMotor rightBackMotor;
    private IMU imu;


    private DcMotor pivotL;
    private DcMotor pivotR;
    private DcMotor telescopeF;
    private DcMotor telescopeB;

    private CRServo intake;
    private CRServo wristL;
    private CRServo wristR;



    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");


        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
        leftFrontMotor  = hardwareMap.get(DcMotor.class, "leftFront");
        leftBackMotor  = hardwareMap.get(DcMotor.class, "leftBack");
        rightFrontMotor = hardwareMap.get(DcMotor.class, "rightFront");
        rightBackMotor = hardwareMap.get(DcMotor.class, "rightBack");

        leftFrontMotor.setDirection(DcMotor.Direction.REVERSE);
        leftBackMotor.setDirection(DcMotor.Direction.REVERSE);
        rightFrontMotor.setDirection(DcMotor.Direction.FORWARD);
        rightBackMotor.setDirection(DcMotor.Direction.FORWARD);



        pivotL = hardwareMap.get(DcMotor.class, "leftPivot");
        pivotR = hardwareMap.get(DcMotor.class, "rightPivot");
        telescopeF = hardwareMap.get(DcMotor.class, "frontTelescope");
        telescopeB = hardwareMap.get(DcMotor.class, "backTelescope");


        pivotL.setDirection(DcMotorSimple.Direction.REVERSE);


        pivotL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        pivotR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telescopeB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        telescopeF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //telescopeF.setDirection(DcMotorSimple.Direction.REVERSE);

        wristL = hardwareMap.get(CRServo.class, "leftWrist");
        wristR = hardwareMap.get(CRServo.class, "rightWrist");
        intake = hardwareMap.get(CRServo.class, "intake");

        wristR.setDirection(CRServo.Direction.REVERSE);





        // Retrieve the IMU from the hardware map
        imu = hardwareMap.get(IMU.class, "imu");
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
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit START
     */
    @Override
    public void init_loop() {
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
/*

Drive controls

 */

// Field centric stuff
        // controller joystick input

        double y = -gamepad1.left_stick_y;
        double x = gamepad1.left_stick_x;
        double rx = gamepad1.right_stick_x;

        // reset heading
        // FIXME adjust to whatever we fw heavy vro
        if (gamepad1.options) {
            imu.resetYaw();
        }

        double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);



        // Rotate the movement direction counter to the bot's rotation
        double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
        double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

        rotX = rotX * 1.1;  // Counteract imperfect strafing


        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio,
        // but only if at least one is out of the range [-1, 1]
        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);

        double frontLeftPower = (rotY + rotX + rx) / denominator;
        double backLeftPower = (rotY - rotX + rx) / denominator;
        double frontRightPower = (rotY - rotX - rx) / denominator;
        double backRightPower = (rotY + rotX - rx) / denominator;

        leftFrontMotor.setPower(frontLeftPower);
        leftBackMotor.setPower(backLeftPower);
        rightFrontMotor.setPower(frontRightPower);
        rightBackMotor.setPower(backRightPower);

    // TURN TO POSITION


    // pivot stuff
        // do that SET_TURNING_MODE thingy
        // while (turning mode = encoder):
        // then turn to encoder
        // while (turning mode = manual):
        // turn while holding

    // arm stuff (2 go bilda)
        // same as pivot
        // run to position for bucket/stow
        // manual extend/retract for grabbing



    // wrist (2 servos)
    // intake (2 servos)


    // MANIPULATORs
        // TODO uncomment and initisalize motors

        // manual pivotd
    if (gamepad2.left_trigger > 0.15) { // lowers pivot
         pivotL.setPower(0.5);
         pivotR.setPower(0.5);
    } else if (gamepad2.right_trigger > 0.15) { // raises pivot
        pivotL.setPower(-0.5);
        pivotR.setPower(-0.5);
    } else {
        pivotL.setPower(0);
        pivotR.setPower(0);
    }


    // TODO make sure to adjust so it doesnt break :(
        // manual arm
    if (gamepad2.a) { // retract
        telescopeB.setPower(0.5);
        telescopeF.setPower(0.5);
    } else if (gamepad2.x) { // extend
        telescopeB.setPower(-0.5);
        telescopeF.setPower(-0.5);
    } else {
        telescopeB.setPower(0);
        telescopeF.setPower(0);
    }


        // manual intake
    if (gamepad2.right_bumper) { // intake
        intake.setPower(1);
    } else if (gamepad2.left_bumper) { // outtake
        intake.setPower(-1);
    } else {
        intake.setPower(0);
    }


        // wrist
    if (gamepad2.y) { // wrist forward
        wristL.setPower(1);
        wristR.setPower(1);
    } else if (gamepad2.b) { // wrist back
        wristL.setPower(-1);
        wristR.setPower(-1);
    } else {
        wristL.setPower(0);
        wristR.setPower(0);
    }


    // run commands

//    if (gamepad1.dpad_down) {
//        stowCommand();
//    }
//
//    if (gamepad1.dpad_left) {
//        intakeCommand();
//    }
//
//    if (gamepad1.dpad_right) {
//        scoreCommand();
//    }

        telemetry.addData("Left pivot encoder position",  pivotL.getCurrentPosition());
        telemetry.addData("Right pivot encoder position",  pivotR.getCurrentPosition());

        telemetry.addData(" ",  " ");

        telemetry.addData("Left telescope encoder position",  telescopeF.getCurrentPosition());
        telemetry.addData("Right telescope encoder position",  telescopeB.getCurrentPosition());

        telemetry.addData(" ", " ");

//        telemetry.addData("Left wrist position", wristL.get);
//        telemetry.addData("Right wrist position", wristR.getPosition());




    }






    // stow command
    public void stowCommand() {
        // TODO stows wrist


        // stow arm
        // FIXME make sure the run at the same time and stuff
        telescopeF.setTargetPosition(0);
        telescopeF.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //FIXME they probably have different target positions to fix that
        telescopeB.setTargetPosition(telescopeF.getTargetPosition());
        telescopeB.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // stow pivot
        // FIXME same deal as arm
        pivotL.setTargetPosition(0);
        pivotL.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        pivotR.setTargetPosition(pivotL.getTargetPosition());
        pivotR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }




    // intake command
    public void intakeCommand() {


        // extend arm
        // FIXME make sure the run at the same time and stuff
        telescopeF.setTargetPosition(100);
        telescopeF.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //FIXME they probably have different target positions to fix that
        telescopeB.setTargetPosition(telescopeF.getTargetPosition());
        telescopeB.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // flip out pivot
        // FIXME same deal as arm
        pivotL.setTargetPosition(200);
        pivotL.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        pivotR.setTargetPosition(pivotL.getTargetPosition());
        pivotR.setMode(DcMotor.RunMode.RUN_TO_POSITION);



        // flip wrist out wrist

        wristR.setPower(-1);
        wristR.setPower(-1);
    }

    // score command
    public void scoreCommand() {


        // extend arm
        // FIXME make sure the run at the same time and stuff
        telescopeF.setTargetPosition(100);
        telescopeF.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //FIXME they probably have different target positions to fix that
        telescopeB.setTargetPosition(telescopeF.getTargetPosition());
        telescopeB.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // flip out pivot
        // FIXME same deal as arm
        pivotL.setTargetPosition(100);
        pivotL.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        pivotR.setTargetPosition(pivotL.getTargetPosition());
        pivotR.setMode(DcMotor.RunMode.RUN_TO_POSITION);



        // flip wrist out wrist
        wristL.setPower(1);
        wristR.setPower(1);
    }


























    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

}




