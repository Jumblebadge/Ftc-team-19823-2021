package org.firstinspires.ftc.teamcode.opmodes;

public class angleWrap {
    /**
     *
     * @param radians
     * @return
     */

    public static double angleWrap(double radians){

        while (radians > Math.PI) {
            radians -= 2 * Math.PI;
        }

        while (radians < -Math.PI) {
            radians += 2 * Math.PI;
        }

        return radians;
    }


}
