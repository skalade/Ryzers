#include "cvml_ros/face_mesh_node.hpp"
#include <opencv2/opencv.hpp>

namespace cvml_ros
{

FaceMeshNode::FaceMeshNode(const rclcpp::NodeOptions & options)
: Node("face_mesh_node", options)
{
  // Declare parameters
  this->declare_parameter("input_topic", "/camera/image_raw");
  this->declare_parameter("output_topic", "/face_mesh/output");
  this->declare_parameter("fd_model_type", "precise");

  input_topic_ = this->get_parameter("input_topic").as_string();
  output_topic_ = this->get_parameter("output_topic").as_string();
  fd_model_type_ = this->get_parameter("fd_model_type").as_string();

  RCLCPP_INFO(this->get_logger(), "Input topic: %s", input_topic_.c_str());
  RCLCPP_INFO(this->get_logger(), "Output topic: %s", output_topic_.c_str());
  RCLCPP_INFO(this->get_logger(), "Face detection model: %s", fd_model_type_.c_str());

  try {
    // Create CVML context
    context_ = amd::cvml::CreateContext();
    if (!context_) {
      throw std::runtime_error("Failed to create CVML context");
    }

    // Set AUTO backend (Face Mesh requires NPU)
    context_->SetInferenceBackend(amd::cvml::Context::InferenceBackend::AUTO);

    // Set streaming mode for real-time processing
    context_->SetStreamingMode(amd::cvml::Context::StreamingMode::ONLINE_STREAMING);

    // Determine face detection model type
    amd::cvml::FaceDetector::FDModelType model_type =
      (fd_model_type_ == "fast") ?
        amd::cvml::FaceDetector::FDModelType::Fast :
        amd::cvml::FaceDetector::FDModelType::Precise;

    // Create face detector
    face_detector_ = std::make_unique<amd::cvml::FaceDetector>(context_, model_type);

    // Create face mesh
    face_mesh_ = std::make_unique<amd::cvml::FaceMesh>(context_);
    face_mesh_->SetMaxNumFaces(-1);  // Process all detected faces

    RCLCPP_INFO(this->get_logger(), "Ryzen AI Face Mesh initialized");
  } catch (const std::exception & e) {
    RCLCPP_ERROR(this->get_logger(), "Failed to initialize Ryzen AI: %s", e.what());
    throw;
  }

  // Create publisher
  output_pub_ = this->create_publisher<sensor_msgs::msg::Image>(output_topic_, 10);
  RCLCPP_INFO(this->get_logger(), "Created publisher on: %s", output_topic_.c_str());

  // Create subscriber
  image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
    input_topic_, 10,
    std::bind(&FaceMeshNode::imageCallback, this, std::placeholders::_1));

  RCLCPP_INFO(this->get_logger(), "Created subscriber on: %s", input_topic_.c_str());
  RCLCPP_INFO(this->get_logger(), "Face mesh node started successfully");
}

FaceMeshNode::~FaceMeshNode()
{
  if (context_) {
    context_->Release();
  }
}

void FaceMeshNode::imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr & msg)
{
  try {
    // Convert ROS2 image to OpenCV
    cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::RGB8);
    cv::Mat input_image = cv_ptr->image;
    cv::Mat output_image = input_image.clone();

    RCLCPP_INFO_ONCE(this->get_logger(), "Received first image: %dx%d", input_image.cols, input_image.rows);

    // Create CVML input image
    amd::cvml::Image amd_img(
      amd::cvml::Image::Format::kRGB,
      amd::cvml::Image::DataType::kUint8,
      input_image.cols,
      input_image.rows,
      input_image.data
    );

    // Detect faces
    auto faces = face_detector_->Detect(amd_img);

    // Process each detected face
    double small_face_size_thr = 0.05;  // Skip faces smaller than 5% of frame width
    int scale_size = 360;
    cv::Scalar point_color(0, 0, 255);  // Red in RGB

    for (size_t i = 0; i < faces.size(); ++i) {
      auto face_width = faces[i].face_.width_;
      if (static_cast<double>(face_width) / static_cast<double>(input_image.cols) < small_face_size_thr) {
        continue;  // Skip small faces
      }

      // Generate mesh for this face
      auto mesh = face_mesh_->CreateMesh(amd_img, faces[i]);

      // Draw landmarks on output image
      for (size_t j = 0; j < mesh.landmarks_.size(); ++j) {
        const auto & landmark = mesh.landmarks_[j];
        cv::Point point(static_cast<int>(landmark.x_), static_cast<int>(landmark.y_));
        int radius = static_cast<int>(input_image.rows / scale_size);
        cv::circle(output_image, point, radius, point_color, radius, cv::FILLED);
      }
    }

    RCLCPP_INFO_ONCE(this->get_logger(), "Processed first frame with %zu faces", faces.size());

    // Convert to ROS2 message
    std_msgs::msg::Header header;
    header.stamp = msg->header.stamp;
    header.frame_id = msg->header.frame_id;

    sensor_msgs::msg::Image::SharedPtr output_msg = cv_bridge::CvImage(
      header,
      sensor_msgs::image_encodings::RGB8,
      output_image
    ).toImageMsg();

    // Publish output
    output_pub_->publish(*output_msg);
    RCLCPP_INFO_ONCE(this->get_logger(), "Published first output image");

  } catch (const std::exception & e) {
    RCLCPP_ERROR(this->get_logger(), "Error processing image: %s", e.what());
  }
}

}  // namespace cvml_ros
