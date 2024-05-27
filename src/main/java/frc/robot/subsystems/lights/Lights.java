package frc.robot.subsystems.lights;

import java.util.Arrays;
import edu.wpi.first.wpilibj.PWM;

public class Lights {

    public static PWM redLED;
    public static PWM greenLED;
    public static PWM blueLED;
    public static double[] rgbValues;

  /**
   * SET THE LIGHTS TO A SOLID COLOR VALUE
   * @param rgbValues THE DOUBLE ARRAY OF THE RGB VALUES (RED, GREEN AND BLUE) 
   */
    public static void makeLightsColors(double[] rgbValues) {
        redLED = new PWM(1); //THESE ARE PLACEHOLDER PORTS
        greenLED = new PWM(2);
        blueLED = new PWM(3);

        redLED.setPosition(rgbValues[0] * (4096/255));
        greenLED.setPosition(rgbValues[1] * (4096/255));
        blueLED.setPosition(rgbValues[2] * (4096/255));
    }

    //SOLID COLOR
    public static void solidColor(double[] rgbValues) {
        makeLightsColors(rgbValues);
    }

      /**
   * SET THE LIGHTS TO CYCLE THROUGH THREE COLORS WITH A CERTAIN DURATION
   * @param colors           THE DOUBLE ARRAY COLORS (COLOR 1, COLOR 2 AND COLOR 3), ALL COMPOSED OF INDIVIDUAL RGB DOUBLE ARRAYS
   * @param durationInMillis THE DURATION OF EACH COLOR
   */
    public static void colorCycle(double[][] colors, int durationInMillis) {
        int numSteps = colors.length;
        for (int i = 0; i < numSteps; i++) {
            makeLightsColors(colors[i]);
            delay(durationInMillis / numSteps);
        }
    }

      /**
   * SET THE LIGHTS TO FLASH TWO DIFFERENT COLORS WITH A SET DURATION AND DELAY
   * @param rgbValues1            THE FIRST RGB VALUE TO FLASH
   * @param rgbValues2            THE SECOND RGB VALUE TO FLASH
   * @param flashDurationInMillis THE DURATION OF THE FLASH (ALSO THE PERIOD IN BETWEEN FLASHES X2)
   */
    public static void colorFlash(double[] rgbValues1, double[] rgbValues2, int flashDurationInMillis) {
        makeLightsColors(rgbValues1);
        delay(flashDurationInMillis / 2);
        makeLightsColors(rgbValues2);
        delay(flashDurationInMillis / 2);
    }

    //COLOR PULSE
    public static void colorPulse(double[] rgbValues, int pulseDurationInMillis) {
        double[] originalColor = rgbValues.clone();
        for (int i = 0; i <= 255; i++) {
            double factor = Math.abs(Math.sin(Math.toRadians(i))) * 255;
            double[] newColor = {originalColor[0] * factor / 255, originalColor[1] * factor / 255, originalColor[2] * factor / 255};
            makeLightsColors(newColor);
            delay(pulseDurationInMillis / 255);
        }
    }

    //COLOR FLICKER
    public static void colorFlicker(double[] rgbValues, int flickerDurationInMillis) {
        while (true) {
            double flickerRed = Math.random() * rgbValues[0];
            double flickerGreen = Math.random() * rgbValues[1];
            double flickerBlue = Math.random() * rgbValues[2];
            double[] flickerColor = {flickerRed, flickerGreen, flickerBlue};
            makeLightsColors(flickerColor);
            delay(flickerDurationInMillis);
        }
    }

    //COLOR STROBE
    public static void colorStrobe(double[] rgbValues, int strobeDurationInMillis) {
        while (true) {
            makeLightsColors(rgbValues);
            delay(strobeDurationInMillis / 2);
            makeLightsColors(new double[]{0, 0, 0}); // Turn off
            delay(strobeDurationInMillis / 2);
        }
    }

    //COLOR FADE
    public static void colorFade(double[] startColor, double[] endColor, int fadeDurationInMillis) {
        int numSteps = 100; // Adjust according to the desired smoothness of fade
        for (int i = 0; i <= numSteps; i++) {
            double[] currentColor = {
                startColor[0] + (endColor[0] - startColor[0]) * i / numSteps,
                startColor[1] + (endColor[1] - startColor[1]) * i / numSteps,
                startColor[2] + (endColor[2] - startColor[2]) * i / numSteps
            };
            makeLightsColors(currentColor);
            delay(fadeDurationInMillis / numSteps);
        }
    }

    //COLOR BURST
    public static void colorBurst(double[] rgbValues, double[] burstColor, int burstDurationInMillis) {
        makeLightsColors(burstColor);
        delay(burstDurationInMillis);
        makeLightsColors(rgbValues);
    }

    //COLOR SPARKLE
    public static void colorSparkle(double[] rgbValues, int sparkleDurationInMillis) {
        while (true) {
            int ledIndex = (int) (Math.random() * rgbValues.length);
            double[] sparkleColor = {Math.random() * rgbValues[0], Math.random() * rgbValues[1], Math.random() * rgbValues[2]};
            double[] currentColor = Arrays.copyOf(rgbValues, rgbValues.length);
            currentColor[ledIndex] = sparkleColor[ledIndex]; //USE SPARKLECOLOR FOR THE SELECTED LED
            makeLightsColors(currentColor);
            delay(sparkleDurationInMillis);
            makeLightsColors(rgbValues);
        }
    }

    //COLOR WAVE
    public static void colorWave(double[][] colors, int waveDurationInMillis) {
        int numColors = colors.length;
        for (int i = 0; i < numColors; i++) {
            makeLightsColors(colors[i]);
            delay(waveDurationInMillis / numColors);
        }
    }

   public static void colorHueSparkle(double[] rgbValues, int sparkleDurationInMillis) {
        while (true) {
            double[] hsb = rgbToHSB(rgbValues[0], rgbValues[1], rgbValues[2]); //CALCULATE HUE AND SATURATION OF ORIGINAL COLOR
            double hue = hsb[0];
            double saturation = hsb[1];
            
            //CREATE RANDOM BRIGHTNESS VALUE
            double brightness = Math.random() * 255;
            
            // GENERATE RANDOM HUE VARIATION WITHIN A RANGE (E.G., +/- 30 DEGREES)
            double hueVariation = (Math.random() * 60) - 30; //ADJUST RANGE AS NEEDED
            
            // CALCULATE NEW RGB VALUES BASED ON ORIGINAL HUE, VARIED BRIGHTNESS, AND SATURATION
            double[] sparkleColor = hueSaturationBrightnessToRGB(hue + hueVariation, saturation, brightness);
            
            makeLightsColors(sparkleColor); //SET LED STRIP TO SPARKLE COLOR
            delay(sparkleDurationInMillis);
            makeLightsColors(rgbValues); //SET LED STRIP TO ORIGINAL COLOR
        }
    }
    
    //HELPER FUNCTION TO CONVERT RGB TO HSB (HUE, SATURATION, BRIGHTNESS)
    private static double[] rgbToHSB(double r, double g, double b) {
        double[] hsb = new double[3];
        double min = Math.min(Math.min(r, g), b);
        double max = Math.max(Math.max(r, g), b);
        double delta = max - min;
    
        //CALCULATE BRIGHTNESS
        hsb[2] = max;
    
        //CALCULATE SATURATION
        if (max != 0) {
            hsb[1] = delta / max;
        } else {
            hsb[1] = 0;
        }
    
        //CALCULATE HUE
        if (delta == 0) {
            hsb[0] = 0; //NO HUE
        } else if (r == max) {
            hsb[0] = (g - b) / delta;
        } else if (g == max) {
            hsb[0] = 2 + (b - r) / delta;
        } else {
            hsb[0] = 4 + (r - g) / delta;
        }
        hsb[0] *= 60; //CONVERT HUE TO DEGREES
        if (hsb[0] < 0) {
            hsb[0] += 360; //MAKE SURE HUE IS POSITIVE
        }
        return hsb;
    }
    
    //HELPER FUNCTION TO CONVERT HSB (HUE, SATURATION, BRIGHTNESS) TO RGB
    private static double[] hueSaturationBrightnessToRGB(double hue, double saturation, double brightness) {
        double chroma = saturation * brightness;
        double huePrime = hue / 60.0;
        double x = chroma * (1 - Math.abs(huePrime % 2 - 1));
        double r1, g1, b1;
        if (0 <= huePrime && huePrime < 1) {
            r1 = chroma;
            g1 = x;
            b1 = 0;
        } else if (1 <= huePrime && huePrime < 2) {
            r1 = x;
            g1 = chroma;
            b1 = 0;
        } else if (2 <= huePrime && huePrime < 3) {
            r1 = 0;
            g1 = chroma;
            b1 = x;
        } else if (3 <= huePrime && huePrime < 4) {
            r1 = 0;
            g1 = x;
            b1 = chroma;
        } else if (4 <= huePrime && huePrime < 5) {
            r1 = x;
            g1 = 0;
            b1 = chroma;
        } else {
            r1 = chroma;
            g1 = 0;
            b1 = x;
        }
        double m = brightness - chroma;
        return new double[]{(r1 + m) * 255, (g1 + m) * 255, (b1 + m) * 255};
    }


    private static void delay(int milliseconds) {
        try {
            Thread.sleep(milliseconds);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    }
}