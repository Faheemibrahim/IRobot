package april.tag;

import java.io.*;
import java.awt.image.*;
import javax.imageio.*;
import april.tag.TagFamily;
import april.tag.Tag36h11;
import april.tag.TagRenderer;


//javac -cp build/april.jar:build/ src/april/tag/Generate36h11.java -d build/
//java -cp build/:build/april.jar april.tag.Generate36h11


public class Generate36h11 {
    public static void main(String args[]) throws IOException {
        TagFamily tagFamily = new Tag36h11(); // Select the tag family
        TagRenderer renderer = new TagRenderer(tagFamily); // Correct constructor

        // Generate AprilTags with IDs from 0 to 30
        int[] tagIDs = new int[31];
        for (int i = 0; i <= 30; i++) {
            tagIDs[i] = i;
        }

        // Define different sizes for the tags
        int[] sizes = {945};

        


        // Ensure the base output directory exists
        File baseOutputDir = new File("generated_tags");
        if (!baseOutputDir.exists()) {
            baseOutputDir.mkdir();
        }

        for (int size : sizes) {
            // Create a subdirectory for each size
            File sizeDir = new File(baseOutputDir, "size_" + size + "px");
            if (!sizeDir.exists()) {
                sizeDir.mkdir();
            }

            for (int tagID : tagIDs) {
                // Render the AprilTag as an image
                BufferedImage img = renderer.makeImage(tagID);

                // Scale the image to desired size
                BufferedImage resizedImg = resizeImage(img, size, size);

                // Define the output file inside the specific size folder
                File outputfile = new File(sizeDir, "tag36h11_" + tagID + "_" + size + "px.png");

                // Save the image as PNG
                ImageIO.write(resizedImg, "png", outputfile);
                System.out.println("Saved: " + outputfile.getPath());
            }
        }
    }

    // Helper method to resize the image
    private static BufferedImage resizeImage(BufferedImage originalImage, int width, int height) {
        BufferedImage resizedImage = new BufferedImage(width, height, BufferedImage.TYPE_INT_ARGB);
        resizedImage.getGraphics().drawImage(originalImage, 0, 0, width, height, null);
        return resizedImage;
    }
}
