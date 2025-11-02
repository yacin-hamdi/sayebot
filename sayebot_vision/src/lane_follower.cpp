#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>


class LaneFollower: public rclcpp::Node
{
public:
    LaneFollower(): Node("lane_follower")
    {
        
        image_sub_ = create_subscription<sensor_msgs::msg::Image>(
            "/camera", 
            10,
            std::bind(&LaneFollower::cameraCallback, this, std::placeholders::_1)
        );
    }

private:
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;

    cv::Mat gray_threshold(const cv::Mat &img, const int min, const int max){
        cv::Mat gray;
        cv::cvtColor(img, gray, cv::COLOR_BGR2GRAY);
        cv::Mat out;
        cv::inRange(gray, min, max, out);
        return out;
    }

    void cameraCallback(const sensor_msgs::msg::Image::SharedPtr msg){
        cv::Mat frame = cv_bridge::toCvCopy(msg, "bgr8")->image;

        cv::Mat white_mask = gray_threshold(frame, 150, 255);
        

        cv::imshow("white_mask", white_mask);

        // cv::Mat hls_img, hsv_img;
        // cv::cvtColor(frame, hls_img, cv::COLOR_BGR2HLS);
        // cv::cvtColor(frame, hsv_img, cv::COLOR_BGR2HSV);

        

        // cv::Mat white_mask, red_mask1, red_mask2, mask;
        // cv::inRange(hls_img, cv::Scalar(0, 160, 0), cv::Scalar(180, 255, 100), white_mask);

        // cv::inRange(hsv_img, cv::Scalar(160, 70, 50), cv::Scalar(180, 255, 255), red_mask1);
        // cv::inRange(hsv_img, cv::Scalar(0, 70, 50), cv::Scalar(10, 255, 255), red_mask2);

        // mask = white_mask | red_mask1 | red_mask2;

    
        // cv::imshow("mask", mask);
        // cv::imshow("red_mask", red_mask1);
        // cv::imshow("white_mask", white_mask);

        // cv::Mat gray, sobelx, abs_sobelx, scaled_sobel;
        // cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
        // cv::Sobel(gray, sobelx, CV_32F, 1, 0, 3);
        // abs_sobelx = cv::abs(sobelx);
        // double minval, maxval;
        // cv::minMaxLoc(abs_sobelx, &minval, &maxval);
        // if(maxval > 0){
        //     abs_sobelx.convertTo(scaled_sobel, CV_8U, 255.0/maxval);
        // }else {
        //     scaled_sobel = cv::Mat::zeros(abs_sobelx.size(), CV_8U);
        // }

        // cv:: Mat grad_binary;
        // cv::inRange(scaled_sobel, 40, 255, grad_binary);

        // cv::imshow("scaled sobel", scaled_sobel);
        // cv::imshow("grad_binary", grad_binary | mask);

        // cv::Mat gray;
        // cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);

        // cv::Mat blur;
        // cv::GaussianBlur(gray, blur, cv::Size(5, 5), 0);

        // cv::Mat edges;
        // cv::Canny(blur, edges, 50, 150);

        // cv::Mat mask = cv::Mat::zeros(edges.size(), edges.type());
        // std::vector<cv::Point> roi_corners;
        // int width = frame.cols;
        // int height = frame.rows;
        // roi_corners.push_back(cv::Point(0, height));
        // roi_corners.push_back(cv::Point(width, height));
        // roi_corners.push_back(cv::Point(width, height * 0.6));
        // roi_corners.push_back(cv::Point(0, height*0.6));
        // cv::fillConvexPoly(mask, roi_corners, 255);
        // cv::Mat masked_edges;
        // cv::bitwise_and(edges, mask, masked_edges);

        // std::vector<cv::Vec4i> lines;
        // cv::HoughLinesP(masked_edges, lines, 1, CV_PI / 180, 50, 50, 150);

        // cv::Mat line_img = frame.clone();
        // for(const auto& line: lines){
        //     cv::line(line_img, cv::Point(line[0], line[1]), cv::Point(line[2], line[3]), cv::Scalar(0, 255, 0), 3);
        // }

        // cv::imshow("detected_lanes", line_img);
        // cv::imshow("mask", mask);
        // cv::imshow("masked_edges", masked_edges);
        
        
        // cv::imshow("gray", gray);
        // cv::imshow("canny", edges);
        
        
        // cv::imshow("blur", blur);
        // cv::imshow("camera", frame);
        cv::waitKey(1);
    }
};


int main(int argc, char **argv){
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LaneFollower>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    cv::destroyAllWindows();
    return 0;
}