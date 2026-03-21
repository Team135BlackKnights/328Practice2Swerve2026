package frc.robot;

public class Utils {
    /**
     * a mod n
     * @param a
     * @param n
     * @return
     */
    public static double mod(double a, double n){
        return (((a % n) + n) % n);
    }
}
