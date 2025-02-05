package pedroPathing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.gamepad.ButtonReader;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import java.util.List;

@Config
@TeleOp(name = "TeleOpCiupa", group = "Robot")
//@Disabled
public class TeleOpCiupa extends LinearOpMode {
    /////
    private DcMotorEx fata_stanga;
    private DcMotorEx fata_dreapta;
    private DcMotorEx spate_dreapta;
    private  DcMotorEx spate_stanga;
    private  DcMotorEx motor_stanga;
    private DcMotorEx motor_glisiere;
    private DcMotorEx hang1;
    private DcMotorEx hang2;
    private ServoEx cleste;
    private ServoEx servoRotire;

    ////

    final double cleste_inchis   = 0;
    final double cleste_deschis  = 1;

    /////

    private PIDController controller;
    public static double p = 0.02, i = 0, d = 0;
    public  static  double f = 0.2;
    public static double target = 0 ;
    private  final double ticks_in_degree = 2.77;

    ////////

    private PIDController lcontroller;
    public static double lp = 0.08, li = 0, ld = 0.00055;
    public  static  double lf = 0.15;
    public static double ltarget = 0 ;
    private  final double ticks_in_mm = 3.20;

    //////

    private PIDController hang1pid;
    public static double h1p = 0, h1i = 0, h1d = 0;
    public  static  double h1f = 0;
    public static double h1target = 0 ;
    private  final double h1ticks_in_degree = 3.434;

    //////

    private PIDController hang2pid;
    public static double h2p = 0, h2i = 0, h2d = 0;
    public  static  double h2f = 0;
    public static double h2target = 0 ;
    private  final double h2ticks_in_degree = 3.434;
    //////



    double cycletime = 0;
    double looptime = 0;
    double oldtime = 0;



    @Override
    public void runOpMode() throws InterruptedException {
        /////

        GamepadEx driverOp = new GamepadEx(gamepad1);
        GamepadEx toolOp = new GamepadEx(gamepad2);

        ButtonReader sus = new ButtonReader(
                toolOp, GamepadKeys.Button.DPAD_UP
        );


        /////
        controller = new PIDController(p, i, d);
        hang1pid = new PIDController(h1p, h1i, h1d);
        hang2pid = new PIDController(h2p, h2i, h2d);
        lcontroller = new PIDController(lp, li, ld);

        /////

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        /////

        fata_dreapta =hardwareMap.get(DcMotorEx.class, "fata_dreapta");
        fata_stanga = hardwareMap.get(DcMotorEx.class, "fata_stanga");
        spate_dreapta = hardwareMap.get(DcMotorEx.class, "spate_dreapta");
        spate_stanga = hardwareMap.get(DcMotorEx.class, "spate_stanga");

        motor_stanga = hardwareMap.get(DcMotorEx.class, "motor_stanga");
        motor_glisiere = hardwareMap.get(DcMotorEx.class, "motor_glisiere");

        hang1 = hardwareMap.get(DcMotorEx.class, "hang1");
        hang2 = hardwareMap.get(DcMotorEx.class, "hang2");

        cleste = hardwareMap.get(ServoEx.class, "cleste");
        servoRotire = hardwareMap.get(ServoEx.class, "servoRotire");

        /////

        fata_dreapta.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);;
        fata_stanga.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        spate_dreapta.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        spate_stanga.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        fata_stanga.setDirection(DcMotorSimple.Direction.REVERSE);
        spate_stanga.setDirection(DcMotorSimple.Direction.REVERSE);

        motor_stanga.setDirection(DcMotorSimple.Direction.REVERSE);
        motor_glisiere.setDirection(DcMotorSimple.Direction.REVERSE);

        /////



        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);

        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
        ////

        telemetry.addLine("Robot Ready.");
        telemetry.update();
        waitForStart();




        /* Run until the driver presses stop */
        while (opModeIsActive())

        {

            /////
            controller.setPID(p,i,d);
            hang1pid.setPID(h1p, h1i, h1d);
            hang2pid.setPID(h2p, h2i, h2d);
            lcontroller.setPID(lp, li, ld);

            /////

            int armPos = motor_stanga.getCurrentPosition();
            int liftPos = motor_glisiere.getCurrentPosition();
            int hang1Pos = hang1.getCurrentPosition();
            int hang2Pos = hang2.getCurrentPosition();

            /////

            double pid = controller.calculate(armPos, target);
            double h1pid = hang1pid.calculate(hang1Pos, h1target);
            double h2pid = hang2pid.calculate(hang2Pos, h2target);
            double lpid = lcontroller.calculate(liftPos, ltarget);

            /////

            double ff = Math.cos(Math.toRadians(target / ticks_in_degree)) * f;
            double h1ff = h1f;
            double h2ff = h2f;
            double lff = lf;

            /////

            double power = pid + ff;
            double h1power = h1pid + h1ff;
            double h2power = h2pid + h2ff;
            double lpower = lpid + lff;

            //////

            double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
            double x = gamepad1.left_stick_x ; // Counteract imperfect strafing
            double rx = gamepad1.right_stick_x;

            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;

            fata_stanga.setPower(frontLeftPower);
            spate_stanga.setPower(backLeftPower);
            fata_dreapta.setPower(frontRightPower);
            spate_dreapta.setPower(backRightPower);

            /////

            ///// IF - URI

            if (gamepad2.a){
                cleste.setPosition(cleste_inchis);
            }
            if(gamepad2.x){
                cleste.setPosition(cleste_deschis);

            }


            if(gamepad2.b){
                servoRotire.setPosition(0.4);
            }
            if(gamepad2.y) {
                servoRotire.setPosition(0.7);
            }


            if(gamepad2.dpad_up){
                target = 105 * ticks_in_degree;
                if(toolOp.isDown(GamepadKeys.Button.DPAD_UP)){
                    ltarget = 100 * ticks_in_mm;
                }
            }
            if(gamepad2.dpad_down){
                target = 90 * ticks_in_degree;
            }
            if(gamepad2.dpad_right) {
                target = 0;
                ltarget = 0;
            }
            if(gamepad2.dpad_left) {
                target = 25 * ticks_in_degree;
                ltarget = 0;
            }

            if(gamepad2.right_bumper) {
                ltarget = ltarget + 20 * ticks_in_mm;
            }
            else if (gamepad2.left_bumper) {
                ltarget = ltarget - 20 * ticks_in_mm;
            }

            /////

            motor_stanga.setPower(power);
            hang1.setPower(h1power);
            hang2.setPower(h2power);
            motor_glisiere.setPower(lpower);

            /////

            looptime = getRuntime();
            cycletime = looptime-oldtime;
            oldtime = looptime;

            telemetry.update();
        }
    }
}