/**
 * @file overlay_node.cpp
 * @brief A ROS2 node that overlays two .pgm images and displays the blended result.
 *
 * This ROS2 node reads two grayscale .pgm images, resizes the second image if necessary,
 * and blends them together using OpenCV's `addWeighted` function. The result is displayed
 * and saved as a new .pgm image. It uses the `rclcpp` library for ROS2 functionality
 * and `opencv2` for image processing.
 */

#include "rclcpp/rclcpp.hpp"
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>

/**
 * @class OverlayNode
 * @brief ROS2 node that overlays two .pgm images and saves the result.
 * 
 * The `OverlayNode` loads two .pgm images, resizes the second image to match
 * the size of the first if necessary, and blends them together. The resulting
 * image is displayed and optionally saved to a file.
 */
class OverlayNode : public rclcpp::Node
{
public:
    /**
     * @brief Constructor for the OverlayNode.
     *
     * The constructor initializes the node, loads two images, blends them, 
     * displays the result, and optionally saves the blended image.
     */
    OverlayNode() : Node("overlay_node")
    {
        RCLCPP_INFO(this->get_logger(), "Overlay Node has been started.");

        // Load two .pgm images (assuming they are in the same directory as the executable)
        cv::Mat image1 = cv::imread("gazebo_map.pgm", cv::IMREAD_GRAYSCALE);  ///< Background image
        cv::Mat image2 = cv::imread("map.pgm", cv::IMREAD_GRAYSCALE);         ///< Foreground image

        // Check if images are loaded correctly
        if (image1.empty() || image2.empty()) {
            RCLCPP_ERROR(this->get_logger(), "Error: Could not load one or both .pgm images.");
            return;
        }

        // Resize image2 to match the size of image1 if necessary
        if (image1.size() != image2.size()) {
            cv::resize(image2, image2, image1.size());
        }

        // Alpha and beta for blending
        double alpha = 0.7;  ///< Transparency for image1
        double beta = 0.3;   ///< Transparency for image2
        cv::Mat blended_image;

        // Blend the images using addWeighted (for grayscale .pgm images)
        cv::addWeighted(image1, alpha, image2, beta, 0.0, blended_image);

        // Display the blended image
        cv::imshow("Blended PGM Image", blended_image);
        cv::waitKey(0);  // Wait for a key press
        cv::destroyAllWindows();

        // Optionally save the blended image to a new .pgm file
        cv::imwrite("blended_output.pgm", blended_image);
        RCLCPP_INFO(this->get_logger(), "Blended .pgm image saved to 'blended_output.pgm'");
    }
};

/**
 * @brief Main function that initializes the ROS2 system and spins the node.
 *
 * The main function initializes the ROS2 system, creates the OverlayNode, and
 * spins it until the node is manually terminated. After that, it shuts down the ROS2 system.
 * 
 * @param argc Number of command-line arguments.
 * @param argv Array of command-line arguments.
 * @return 0 on successful execution.
 */
int main(int argc, char **argv)
{
    // Initialize the ROS2 system
    rclcpp::init(argc, argv);

    // Create the node
    auto node = std::make_shared<OverlayNode>();

    // Keep spinning the node until it's manually terminated
    rclcpp::spin(node);

    // Shut down the ROS2 system
    rclcpp::shutdown();

    return 0;
}
