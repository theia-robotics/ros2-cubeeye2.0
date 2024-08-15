#ifndef CUBEEYE_CAMERA_NODE_H_
#define CUBEEYE_CAMERA_NODE_H_

#include <rclcpp/rclcpp.hpp>


class CameraModule;

class CubeEyeCameraNode : public rclcpp::Node
{
public:
    CubeEyeCameraNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
    virtual ~CubeEyeCameraNode() = default;

    void init();
    bool shutdown();

protected:
#if 0
    void getLastStateServiceCallback(const std::shared_ptr<cubeeye_camera::srv::LastState::Request> request,
                                    std::shared_ptr<cubeeye_camera::srv::LastState::Response> response);
    void getLastErrorServiceCallback(const std::shared_ptr<cubeeye_camera::srv::LastError::Request> request,
                                    std::shared_ptr<cubeeye_camera::srv::LastError::Response> response);
    void getScanServiceCallback(const std::shared_ptr<cubeeye_camera::srv::Scan::Request> request,
                                    std::shared_ptr<cubeeye_camera::srv::Scan::Response> response);
    void getConnectServiceCallback(const std::shared_ptr<cubeeye_camera::srv::Connect::Request> request,
                                    std::shared_ptr<cubeeye_camera::srv::Connect::Response> response);
    void getRunServiceCallback(const std::shared_ptr<cubeeye_camera::srv::Run::Request> request,
                                    std::shared_ptr<cubeeye_camera::srv::Run::Response> response);
    void getStopServiceCallback(const std::shared_ptr<cubeeye_camera::srv::Stop::Request> request,
                                    std::shared_ptr<cubeeye_camera::srv::Stop::Response> response);
    void getDisconnectServiceCallback(const std::shared_ptr<cubeeye_camera::srv::Disconnect::Request> request,
                                        std::shared_ptr<cubeeye_camera::srv::Disconnect::Response> response);
#endif

private:
    void connectOnInit(int index, bool enable_depth, bool enable_pointcloud);

    rclcpp::Logger mLogger;

#if 0
    rclcpp::Service<cubeeye_camera::srv::LastState>::SharedPtr mLastStateService;
    rclcpp::Service<cubeeye_camera::srv::LastError>::SharedPtr mLastErrorService;
    rclcpp::Service<cubeeye_camera::srv::Scan>::SharedPtr mScanService;
    rclcpp::Service<cubeeye_camera::srv::Connect>::SharedPtr mConnectService;
    rclcpp::Service<cubeeye_camera::srv::Run>::SharedPtr mRunService;
    rclcpp::Service<cubeeye_camera::srv::Stop>::SharedPtr mStopService;
    rclcpp::Service<cubeeye_camera::srv::Disconnect>::SharedPtr mDisconnectService;
#endif

    std::shared_ptr<CameraModule> mCamera;
};


#endif // CubeEyeCameraNode
