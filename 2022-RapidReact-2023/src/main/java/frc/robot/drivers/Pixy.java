package frc.robot.drivers;

import java.util.ArrayList;

import io.github.pseudoresonance.pixy2api.Pixy2;
import io.github.pseudoresonance.pixy2api.Pixy2CCC.Block;
import io.github.pseudoresonance.pixy2api.links.SPILink;

public class Pixy {
    private static Pixy2 pixy;

    private final static byte LED_OFF = (byte) 0;
    private final static byte LED_ON = (byte) 1;
    private final static int LED_MAX_RBG = 255; // full white
    private final static int LED_MIN_RBG = 0;

    public final static int TEAM_RED = 2;
    public final static int TEAM_BLUE = 1;
    private int teamColor;

    private int maxBlocks = 10;

    public Pixy(int teamColor) {
        this.teamColor = teamColor;
        pixy = Pixy2.createInstance(new SPILink());
        pixy.init();
    }

    public void turnOnLights() {
        pixy.setLamp(LED_ON, LED_ON);
        pixy.setLED(LED_MAX_RBG, LED_MAX_RBG, LED_MAX_RBG);
    }

    public void turnOffLights(){
        pixy.setLamp(LED_OFF, LED_OFF);
        pixy.setLED(LED_MIN_RBG, LED_MIN_RBG, LED_MIN_RBG);
    }

    public Block getLargestBlock() {
        int blockCount = pixy.getCCC().getBlocks(false, teamColor, maxBlocks);

        if (blockCount <= 0) {
            return null;
        }

        ArrayList<Block> blocks = pixy.getCCC().getBlockCache();

        Block largestBlock = null;
        for (Block block : blocks) {
            if (largestBlock == null) {
                largestBlock = block;
            } else if (getArea(block) > getArea(largestBlock) && isWithinRange(block) && isSquare(block)) {
                largestBlock = block;
            }
        }
        return largestBlock;
    }

    public boolean isWithinRange(Block block) {
        if (getArea(block) > 100 && getArea(block) < 2500) {
            return true;
        }
        return false;
    }
    
    public boolean isSquare(Block block) {
        double height = block.getHeight();
        double width = block.getWidth();
        double longerSide = Math.max(height, width);
        double shorterSide = Math.max(height,width);

        if (longerSide < 1.25 * shorterSide) {
            return true;
        }
        return false;
    }

    public double getArea(Block block) {
        if (block == null) {
            return 0;
        }
        return block.getWidth() * block.getHeight();
    }

    public double getX(Block block) {
        if (block == null) {
            return 0;
        }
        return block.getX();
    }
}
