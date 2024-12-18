#include <rclcpp/rclcpp.hpp>
#include <metavision/sdk/driver/camera.h>
#include <metavision/sdk/core/utils/cd_frame_generator.h>
#include <opencv2/highgui.hpp>
#include <mutex>

class MetavisionViewerNode : public rclcpp::Node {
public:
    MetavisionViewerNode() : Node("metavision_viewer_node") {
        // パラメータ宣言とデフォルト値設定
        this->declare_parameter<std::string>("input_event_file", "");
        this->declare_parameter<std::string>("bias_file", "");
        this->declare_parameter<std::string>("record_file", "/tmp/output.raw");
        this->declare_parameter<int>("frame_rate_ms", 33);

        // パラメータ取得
        input_event_file_ = this->get_parameter("input_event_file").as_string();
        bias_file_ = this->get_parameter("bias_file").as_string();
        record_file_ = this->get_parameter("record_file").as_string();
        timer_interval_ms_ = this->get_parameter("frame_rate_ms").as_int();

        // ログで操作説明を表示
        log_instructions();

        // カメラ初期化
        init_camera();

        // フレーム更新用のタイマー
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(timer_interval_ms_), 
            std::bind(&MetavisionViewerNode::timer_callback, this)
        );
    }

    ~MetavisionViewerNode() {
        if (is_recording_) {
            stop_recording();
        }
        if (camera_.is_running()) {
            camera_.stop();
        }
    }

private:
    void log_instructions() const {
        RCLCPP_INFO(this->get_logger(), "=== Metavision Viewer Node Instructions ===");
        RCLCPP_INFO(this->get_logger(), "Parameters:");
        RCLCPP_INFO(this->get_logger(), "  input_event_file: %s", input_event_file_.c_str());
        RCLCPP_INFO(this->get_logger(), "  bias_file: %s", bias_file_.c_str());
        RCLCPP_INFO(this->get_logger(), "  record_file: %s", record_file_.c_str());
        RCLCPP_INFO(this->get_logger(), "  frame_rate_ms: %d", timer_interval_ms_);
        RCLCPP_INFO(this->get_logger(), "Controls:");
        RCLCPP_INFO(this->get_logger(), "  q / ESC: Quit the application");
        RCLCPP_INFO(this->get_logger(), "  r: Start/Stop recording events");
        RCLCPP_INFO(this->get_logger(), "===========================================");
    }

    void init_camera() {
        try {
            // 入力ファイルまたはライブカメラをオープン
            if (!input_event_file_.empty()) {
                RCLCPP_INFO(this->get_logger(), "Opening event file: %s", input_event_file_.c_str());
                camera_ = Metavision::Camera::from_file(input_event_file_);
            } else {
                RCLCPP_INFO(this->get_logger(), "Opening camera from first available device.");
                camera_ = Metavision::Camera::from_first_available();
            }

            // バイアスファイルの適用
            if (!bias_file_.empty()) {
                RCLCPP_INFO(this->get_logger(), "Loading bias file: %s", bias_file_.c_str());
                camera_.biases().set_from_file(bias_file_);
            } else {
                RCLCPP_WARN(this->get_logger(), "No bias file specified. Using default biases.");
            }

            // カメラスタート
            auto geometry = camera_.geometry();
            cd_frame_generator_ = std::make_unique<Metavision::CDFrameGenerator>(geometry.width(), geometry.height());
            cd_frame_generator_->set_display_accumulation_time_us(10000);

            camera_.cd().add_callback(
                [this](const Metavision::EventCD *begin, const Metavision::EventCD *end) {
                    std::lock_guard<std::mutex> lock(cd_frame_mutex_);
                    cd_frame_generator_->add_events(begin, end);
                }
            );

            camera_.start();
            cd_frame_generator_->start(
                30, [this](Metavision::timestamp ts, const cv::Mat &frame) {
                    std::lock_guard<std::mutex> lock(cd_frame_mutex_);
                    frame.copyTo(cd_frame_);
                }
            );
        } catch (const Metavision::CameraException &e) {
            RCLCPP_ERROR(this->get_logger(), "Failed to initialize camera: %s", e.what());
            rclcpp::shutdown();
        }
    }

    void timer_callback() {
        std::lock_guard<std::mutex> lock(cd_frame_mutex_);
        if (!cd_frame_.empty()) {
            cv::imshow("CD Events", cd_frame_);
            int key = cv::waitKey(1);
            handle_key_input(key);
        }
    }

    void handle_key_input(int key) {
        if (key == 'q' || key == 27) {
            rclcpp::shutdown();
        } else if (key == 'r') {
            if (is_recording_) {
                stop_recording();
            } else {
                start_recording();
            }
        }
    }

    void start_recording() {
        try {
            camera_.start_recording(record_file_);
            RCLCPP_INFO(this->get_logger(), "Started recording to: %s", record_file_.c_str());
            is_recording_ = true;
        } catch (const Metavision::CameraException &e) {
            RCLCPP_ERROR(this->get_logger(), "Failed to start recording: %s", e.what());
        }
    }

    void stop_recording() {
        try {
            camera_.stop_recording();
            RCLCPP_INFO(this->get_logger(), "Stopped recording.");
            is_recording_ = false;
        } catch (const Metavision::CameraException &e) {
            RCLCPP_ERROR(this->get_logger(), "Failed to stop recording: %s", e.what());
        }
    }

    rclcpp::TimerBase::SharedPtr timer_;
    int timer_interval_ms_;
    std::string input_event_file_;
    std::string bias_file_;
    std::string record_file_;
    bool is_recording_ = false;
    Metavision::Camera camera_;
    std::unique_ptr<Metavision::CDFrameGenerator> cd_frame_generator_;
    std::mutex cd_frame_mutex_;
    cv::Mat cd_frame_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MetavisionViewerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
