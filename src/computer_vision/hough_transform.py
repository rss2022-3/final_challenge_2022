import cv2
import numpy as np

input_filepath = "src/computer_vision/test_images_input/"
output_filepath = "src/computer_vision/test_images_output/"

def draw_line(img, rho, theta, color, thickness = 1):
    height = img.shape[0]
    bottom = rho/np.cos(theta)
    top = (rho/np.sin(theta)-height)*np.tan(theta)
    a = np.cos(theta)
    b = np.sin(theta)
    x0 = a*rho
    y0 = b*rho
    x1 = int(x0 + 1000*(-b))
    y1 = int(y0 + 1000*(a))
    x2 = int(x0 - 1000*(-b))
    y2 = int(y0 - 1000*(a))
    cv2.line(img,(x1,y1),(x2,y2),color,thickness)

def get_trajectory(img, filename = "trajectory.jpg"):
    img = img[int(img.shape[0]/3):img.shape[0], :]
    height = img.shape[0]
    gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)  

    lower_threshold = 350 #TUNE
    upper_threshold = 500 #TUNE
    edges = cv2.Canny(gray,lower_threshold,upper_threshold,apertureSize = 3) 
    #cv2.imwrite(output_filepath+"canny.jpg", edges) 

    min_intersections = 120 #TUNE                         
    lines = cv2.HoughLines(edges,1,np.pi/180,min_intersections)    

    min_left_theta, max_right_theta = np.Inf, 0
    left, right = None, None
    for i in range(lines.shape[0]):                         
        for rho,theta in lines[i]:
            if rho > 0 and theta < min_left_theta:
                min_left_theta = theta
                left = (rho,theta)
            elif rho < 0 and theta > max_right_theta:
                max_right_theta = theta
                right = (rho,theta)
            draw_line(img, rho, theta, (0,0,255))

    draw_line(img, left[0], left[1], (0,255,0))
    draw_line(img, right[0], right[1], (0,255,0))
    #cv2.imwrite(output_filepath+"hough.jpg", img)

    bottom = (left[0]/np.cos(left[1]) + right[0]/np.cos(right[1]))/2
    top = ((left[0]/np.sin(left[1])-height)*np.tan(left[1]) + 
            (right[0]/np.sin(right[1])-height)*np.tan(right[1]))/2

    cv2.line(img,(int(bottom),0),(int(top),height),(255,0,0),1)
    cv2.imwrite(output_filepath+filename, img)

    return ((bottom, 0), (top, height))


def test_hough(filename):
    img = cv2.imread(input_filepath+filename)
    return get_trajectory(img, filename)

for i in range(6):
    filename = "image ("+str(i+1)+").png"
    try:
        test_hough(filename)
        print(filename+" success")
    except:
        print(filename+" fail")
    