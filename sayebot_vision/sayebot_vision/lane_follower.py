#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np



class LaneFollower(Node):
    def __init__(self):
        super().__init__("lane_follower")
        self.bridge = CvBridge()
        self.image_sub_ = self.create_subscription(Image, 
                                                   "/camera", 
                                                   self.image_callback,
                                                   10)
        
    
    def gray_threshold(self, img, threshold=(200, 255)):
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        binary_out = cv2.inRange(gray, threshold[0], threshold[1])
        return binary_out
    
    def color_threshold(self, img, threshold=(10, 200)):
        hls = cv2.cvtColor(img, cv2.COLOR_BGR2HLS)
        L = hls[:, :, 2]
        binary_output = cv2.inRange(L, threshold[0], threshold[1])
        return binary_output
    
    def dir_gradient(self, img, kernel=3, threshold=(0.7, 1.3)):
        hls = cv2.cvtColor(img, cv2.COLOR_BGR2HLS)
        sobelx = cv2.Sobel(hls, cv2.CV_64F, 1, 0, ksize=kernel)
        sobely = cv2.Sobel(hls, cv2.CV_64F, 0, 1, ksize=kernel)
        dir_sobelxy = np.arctan2(np.absolute(sobely), np.absolute(sobelx))
        binary_output = cv2.inRange(dir_sobelxy, threshold[0], threshold[1])
        return binary_output
    
    def mag_gradient(self, img, kernel=3, threshold=(100, 255)):
        hls = cv2.cvtColor(img, cv2.COLOR_BGR2HLS)
        sobelx = cv2.Sobel(hls, cv2.CV_64F, 1, 0, ksize=kernel)
        sobely = cv2.Sobel(hls, cv2.CV_64F, 0, 1, ksize=kernel)
        abs_sobelxy = np.sqrt(np.power(sobelx, 2) + np.power(sobely, 2))
        scaled_gradient = np.uint8(255*abs_sobelxy/np.max(abs_sobelxy))
        binary_output = cv2.inRange(scaled_gradient, threshold[0], threshold[1])
        return binary_output, scaled_gradient
    
    def abs_sobel_thres(self, img, orient='x', kernel=3, threshold=(0.7, 1.3)):
        hls = cv2.cvtColor(img, cv2.COLOR_BGR2HLS)
        if orient == 'x':
            sobel = cv2.Sobel(hls, cv2.CV_64F, 1, 0, ksize=kernel)
        elif orient == 'y':
            sobel = cv2.Sobel(hls, cv2.CV_64F, 0, 1, ksize=kernel)

        abs_sobel = np.absolute(sobel)
        scaled_sobel = np.uint8(255*abs_sobel/np.max(abs_sobel))

        binary_output = cv2.inRange(scaled_sobel, threshold[0], threshold[1])
        return binary_output
    
    def warp_corners(self, c_img):
        offset = 100
        # c_img = crop_image(img, offset)
        img_size = (c_img.shape[1], c_img.shape[0])
        
        
        src = np.float32([[(img_size[0]//2)-offset, (img_size[1])//2+offset], 
                        [(img_size[0])//2+offset, (img_size[1])//2+offset], 
                        [0, img_size[1]], 
                        [img_size[0], img_size[1]]])

        dst = np.float32([[0, 0], 
                        [img_size[0], 0],
                        [offset, img_size[1]],   
                        [img_size[0]-offset, img_size[1]]])

        M = cv2.getPerspectiveTransform(src, dst)
        Minv = cv2.getPerspectiveTransform(dst, src)
        warped = cv2.warpPerspective(c_img, M, img_size)
        return warped, M, Minv



    def undistort_img(img, objpoints, imgpoints):
        ret, mtx, dist, rvecs, tvecs  = cv2.calibrateCamera(objpoints, imgpoints, img.shape[1::-1], None, None)
        u_img = cv2.undistort(img, mtx, dist, None, mtx)
        return u_img, mtx, dist




    def find_lanes(self, binary_image):
        ym_per_pix = 30/720 
        xm_per_pix = 3.7/700
        
        histogram = np.sum(binary_image[binary_image.shape[0]//2:, :], axis=0)
        out_image = np.dstack((binary_image, binary_image, binary_image))
        midpoint = np.int32(histogram.shape[0]//2)
        leftx_base = np.argmax(histogram[:midpoint])
        rightx_base = np.argmax(histogram[midpoint:]) + midpoint
        # plt.plot(histogram)
        # plt.plot(midpoint, 0, '.')
        # plt.plot(leftx_base, 0, '.')
        # plt.plot(rightx_base, 0, '.')
        nwindows = 9
        margin = 100
        minpix = 50
        
        window_height = np.int32(binary_image.shape[0]//nwindows)
        nonzero = binary_image.nonzero()
        nonzeroy = np.array(nonzero[0])
        nonzerox = np.array(nonzero[1])
        
        leftx_current = leftx_base
        rightx_current = rightx_base
        
        left_lane_inds = []
        right_lane_inds = []
        
        for window in range(nwindows):
            win_y_low = binary_image.shape[0] - (window+1)*window_height
            win_y_high = binary_image.shape[0] - window * window_height
            ### TO-DO: Find the four below boundaries of the window ###
            win_xleft_low = leftx_current - margin  # Update this
            win_xleft_high = leftx_current + margin  # Update this
            win_xright_low = rightx_current - margin  # Update this
            win_xright_high = rightx_current + margin  # Update this
            
            # cv2.rectangle(out_image, (win_xleft_low, win_y_low), (win_xleft_high, win_y_high), (0,255, 0), 2)
            # cv2.rectangle(out_image, (win_xright_low, win_y_low), (win_xright_high, win_y_high), (0, 255, 0), 2)
            
            ### TO-DO: Identify the nonzero pixels in x and y within the window ###
            good_left_inds = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) & (nonzerox >= win_xleft_low) & (nonzerox < win_xleft_high)).nonzero()[0]
            good_right_inds = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) & (nonzerox >= win_xright_low) & (nonzerox < win_xright_high)).nonzero()[0]
            
            left_lane_inds.append(good_left_inds)
            right_lane_inds.append(good_right_inds)
            
            ### TO-DO: If you found > minpix pixels, recenter next window ###
            ### (`right` or `leftx_current`) on their mean position ###
            if len(good_left_inds) > minpix:
                leftx_current = np.int32(np.mean(nonzerox[good_left_inds]))
            
            if len(good_right_inds) > minpix:
                rightx_current = np.int32(np.mean(nonzerox[good_right_inds]))
            
            
        try:
            left_lane_inds = np.concatenate(left_lane_inds)
            right_lane_inds = np.concatenate(right_lane_inds)
        except ValueError:
            # Avoids an error if the above is not implemented fully
            pass
        
        # Extract left and right line pixel positions
        leftx = nonzerox[left_lane_inds]
        lefty = nonzeroy[left_lane_inds] 
        rightx = nonzerox[right_lane_inds]
        righty = nonzeroy[right_lane_inds]
        # Fit a second order polynomial to each
        left_fit = np.polyfit(lefty, leftx, 2)
        right_fit = np.polyfit(righty, rightx, 2)
        
        # Fit a second order polynomial to each
        left_fit_m = np.polyfit(lefty*ym_per_pix, leftx*xm_per_pix, 2)
        right_fit_m = np.polyfit(righty*ym_per_pix, rightx*xm_per_pix, 2)
        
        out_image[lefty, leftx] = [255, 0, 0]
        out_image[righty, rightx] = [0, 0, 255]
        
        # return (left_fit, right_fit, left_fit_m, right_fit_m, left_lane_inds, right_lane_inds, out_img, nonzerox, nonzeroy)

        return left_fit, right_fit, left_fit_m, right_fit_m, out_image





    def draw_rectangle(self, u_img,warp_img, Minv, left_fit, right_fit):
        
        yMax = u_img.shape[0]
        ploty = np.linspace(0, yMax - 1, yMax)
        left_fitx = left_fit[0]*ploty**2 + left_fit[1]*ploty + left_fit[2]
        right_fitx = right_fit[0]*ploty**2 + right_fit[1]*ploty + right_fit[2]
        # Create an image to draw the lines on
        warp_zero = np.zeros_like(warp_img).astype(np.uint8)
        color_warp = np.dstack((warp_zero, warp_zero, warp_zero))

        # Recast the x and y points into usable format for cv2.fillPoly()
        pts_left = np.array([np.transpose(np.vstack([left_fitx, ploty]))])
        pts_right = np.array([np.flipud(np.transpose(np.vstack([right_fitx, ploty])))])
        pts = np.hstack((pts_left, pts_right))

        # Draw the lane onto the warped blank image
        cv2.fillPoly(color_warp, np.int_([pts]), (0,255, 0))

        # Warp the blank back to original image space using inverse perspective matrix (Minv)
        newwarp = cv2.warpPerspective(color_warp, Minv, (u_img.shape[1], u_img.shape[0])) 
        # Combine the result with the original image
        result = cv2.addWeighted(u_img, 1, newwarp, 0.3, 0)
        return result


    def calculateCurvature(y_eval, left_fit_cr, right_fit_cr):
        
        ym_per_pix = 30/720 
        left_curvature = ((1 + (2*left_fit_cr[0]*y_eval*ym_per_pix + left_fit_cr[1])**2)**1.5) / np.absolute(2*left_fit_cr[0])
        right_curvature = (1+ (2*right_fit_cr[0]*y_eval*ym_per_pix + right_fit_cr[1])**2)**1.5/np.abs(2*right_fit_cr[0])
        return left_curvature, right_curvature


    def drawCurvature(img, left_curv, right_curv):
        img_cp = np.copy(img)
        font = cv2.FONT_HERSHEY_SIMPLEX
        fontColor = (255, 255, 255)
        cv2.putText(img_cp, 'Left curvature: {:.0f} m'.format(left_curv), (50, 50), font, 2, fontColor, 2)
        cv2.putText(img_cp, 'Right curvature: {:.0f} m'.format(right_curv), (50, 120), font, 2, fontColor, 2)
        return img_cp




    def image_callback(self, msg: Image):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        h, w, _ = frame.shape
        # roi = frame[int(h*0.5):, :]
        # hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        # mask_white = cv2.inRange(hsv, (0, 0, 200), (180, 40, 255))
        # # kernel = np.ones((5, 5), np.uint8)
        # # mask_white = cv2.dilate(mask_white, kernel, iterations=1)

        # contours, _ = cv2.findContours(mask_white, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        # if contours:
        #     all_pts = np.vstack(contours).squeeze()

        #     if len(all_pts) > 10:
        #         vx, vy, x0, y0 = cv2.fitLine(all_pts, cv2.DIST_L2, 0, 0.01, 0.01)

        #     h, w = mask_white.shape[:2]
        #     y_bottom = h-1
        #     x_bottom = int(x0 + (y_bottom - y0) * (vx / vy))

        #     cv2.line(mask_white, 
        #              (x_bottom, y_bottom), 
        #              (int(x0 - 100 * vx), int(y0 - 100 * vy)), 
        #              (0, 255, 0), 
        #              2)
        #     cv2.circle(mask_white, (x_bottom, y_bottom), 5, (0, 0, 255), -1)
        sobel = self.abs_sobel_thres(frame, kernel=3, threshold=(10,100))
        mg = self.mag_gradient(frame, threshold=(70, 100))
        dir = self.dir_gradient(frame, threshold=(0.8, 1.3))
        c = self.color_threshold(frame, threshold=(160, 255))
        g = self.gray_threshold(frame)
        binary_out = np.zeros_like(g)
        binary_out[((g == 255)|(c == 255))] = 255

        warp_img, M, Minv = self.warp_corners(binary_out[:int(binary_out.shape[0]*0.6), :])
        left_fit, right_fit, left_fit_m, right_fit_m, out_img = self.find_lanes(warp_img)
        out_img = self.draw_rectangle(frame, warp_img, Minv, left_fit, right_fit)




            
        cv2.imshow("warp_img", binary_out[:int(binary_out.shape[0]*0.6), :])
        cv2.imshow("out", out_img)
        # cv2.imshow("gray", g)
        # cv2.imshow("out", binary_out)
       

        # cv2.imshow("frame", frame)
        cv2.waitKey(1)




def main():
    rclpy.init()
    node = LaneFollower()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()

