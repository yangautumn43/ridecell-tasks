#!/usr/bin/python

import cv2
import sys

class ImagePoints():
    def __init__(self, filename):
        """ To select points of interest in the image
        filename : the path to the image file
        """
        self.image = cv2.imread(filename)
        self.points = []

        print("Received image", self.image.shape, filename )
        print("\nClick to select the points, and press space key to show the results\n")
    
    def mouseCallback(self, event, x, y, flags, params):
        """ Every left-click up, record the point
        """
        if event == cv2.EVENT_LBUTTONUP:
            self.points.append((x,y))
            print (x, y)

    def selectImagePoints(self):
        """ Select points of interest in the image,
        save to point array for output
        """
        cv2.namedWindow("win")
        cv2.setMouseCallback("win", self.mouseCallback)

        while True:
            cv2.imshow("win", self.image)

            # get key from keyboard to stop
            key = cv2.waitKey(1) & 0xFF

            if key == ord(" "):
                break

    def showImagePoints(self):
        """ Display the points selected and save the results to a file
        """
        for point in self.points:
            cv2.circle(self.image, point, 3, (0, 0, 180), 2)
        cv2.imwrite("image_points.png", self.image)
        cv2.imshow("win", self.image)

        print("\nPress any key to quit ...")


if __name__ == "__main__":
    # Get fname from command line (cmd line input required)
    if len(sys.argv) > 1:
        filename = sys.argv[1]
    else:
        print('Usage:')
        print(sys.argv[0] + ' image_file.png (input image file)')
        sys.exit(1)         

    ip = ImagePoints(filename)
    ip.selectImagePoints()
    ip.showImagePoints()

    # press any key to quit the program
    q = cv2.waitKey(0)
    cv2.destroyAllWindows()



