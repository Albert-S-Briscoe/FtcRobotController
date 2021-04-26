package org.firstinspires.ftc.teamcode;

import com.vuforia.Image;
import com.vuforia.PIXEL_FORMAT;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;

import java.nio.ByteBuffer;

public class RingCounter {
    
    VuforiaLocalizer vuforia;
    VuforiaLocalizer.CloseableFrame frame = null;
    ByteBuffer buffer;
    final static int[] TEST_RANGE = {440, 230, 640, 380}; // formatted as x1 y1 x2 y2 from the top left corner
    final static double[] COLOR_POSITIVE_RANGE = {17, 32, 0.4, 0.95}; // formatted as hue min, hue max, sat min, sat max
    final static int[] RING_STACK_SIZE_BRACKETS = {100, 1200}; // 1 ring min orange pixels, 4 rings min orange pixels
    int orangePixels;
    
    RingCounter(VuforiaLocalizer vuforia) {
        
        this.vuforia = vuforia;
        
        Vuforia.setFrameFormat(PIXEL_FORMAT.RGB565, true); // enables RGB565 format for the image
        vuforia.setFrameQueueCapacity(1); // tells VuforiaLocalizer to only store one frame at a time
    }
    
    public int count() {
    
        orangePixels = 0;
        int p; // index of pixel
        
        try {
            frame = vuforia.getFrameQueue().take(); //takes the frame at the head of the queue
        } catch (InterruptedException e) {
        
        }
    
        Image rgb = null;
        if (frame != null) {
            long numImages = frame.getNumImages();
        
            for (int i = 0; i < numImages; i++) {
                if (frame.getImage(i).getFormat() == PIXEL_FORMAT.RGB565) {
                    rgb = frame.getImage(i);
                    break;
                }//if
            }//for
        }
        if (rgb != null) {
            buffer = rgb.getPixels();
        } else {
            return -1;
        }
        
        for (int y = TEST_RANGE[1]; y < TEST_RANGE[3]; y++) {
            for (int x = TEST_RANGE[0]; x < TEST_RANGE[2]; x++) {
                
                p = 640*y + x;
                
                if (isOrange(p)) orangePixels++;
                
            }
        }
        
        if (orangePixels > RING_STACK_SIZE_BRACKETS[0]) {
            if (orangePixels > RING_STACK_SIZE_BRACKETS[1]) {
                return 4;
            } else {
                return 1;
            }
        } else {
            return 0;
        }
        
    }
    
    public double[] getHueAndSaturation(int Red, int Green, int Blue) {
        
        double R = (double) (Red)/255;
        double G = (double) (Green)/255;
        double B = (double) (Blue)/255;
        
        double hue;
        double saturation;
    
        //numerator = 0.5*((R-G)+(R-B));
        //denominator = Math.sqrt(Math.pow(R - G, 2) + ((R - B) * (G - B)));
        
        hue = Math.toDegrees(Math.acos(0.5*((R-G)+(R-B))/Math.sqrt(Math.pow(R - G, 2) + ((R - B) * (G - B)))));
        
        if (B > G) hue = 360 - hue;
        
        saturation = 1-(3/(R + G + B)) * Math.min(Math.min(R,G), B);
        
        return new double[] {hue, saturation};
        
    }
    
    private boolean isOrange(int pixel) {
        int pixelHigh = buffer.get(pixel*2 + 1);
        int pixelLow = buffer.get(pixel*2);
        double[] hueSat = getHueAndSaturation( (pixelHigh & 0b11111000 ), ((pixelHigh & 0b00000111)<<5 | (pixelLow & 0b11100000)>>3), ((pixelLow & 0b00011111)<<3));
        return hueSat[0] > COLOR_POSITIVE_RANGE[0] && hueSat[0] <= COLOR_POSITIVE_RANGE[1] && hueSat[1] > COLOR_POSITIVE_RANGE[2] && hueSat[1] <= COLOR_POSITIVE_RANGE[3];
    }
    
    public static String toBinary(int x, int len) {
    
        // Example Use:
        // String pixel = toBinary(((0xFF & buffer.get(614399)) << 8)| (0xFF & buffer.get(614398)), 16);
        // telemetry.addData("pixel color", "R:%s G:%s B:%s", pixel1.substring(0,5),pixel1.substring(5,11),pixel1.substring(11,16));
        
        if (len > 0) {
            return String.format("%" + len + "s", Integer.toBinaryString(x)).replaceAll(" ", "0");
        }
        
        return null;
        
    }
    
}
