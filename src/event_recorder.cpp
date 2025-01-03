#include <rclcpp/rclcpp.hpp>
#include <metavision/sdk/driver/camera.h>
#include <metavision/sdk/core/utils/cd_frame_generator.h>
#include <opencv2/highgui.hpp>
#include <std_msgs/msg/bool.hpp>
#include <filesystem>
#include <mutex>
#include <iomanip>
#include <sstream>
#include <ctime>

class MetavisionViewerNode : public rclcpp::Node {
public:
    MetavisionViewerNode() : Node("metavision_viewer_node") {
        // --- パラメータ宣言とデフォルト値設定 ---
        this->declare_parameter<std::string>("input_event_file", "");
        this->declare_parameter<std::string>("bias_file", "");
        this->declare_parameter<std::string>("record_directory", "/tmp");
        this->declare_parameter<int>("frame_rate_ms", 33);
        this->declare_parameter<int>("rotation_cycle_seconds", 300); // デフォルト5分

        // --- パラメータ取得 ---
        input_event_file_ = this->get_parameter("input_event_file").as_string();
        bias_file_ = this->get_parameter("bias_file").as_string();
        record_directory_ = this->get_parameter("record_directory").as_string();
        timer_interval_ms_ = this->get_parameter("frame_rate_ms").as_int();
        rotation_cycle_seconds_ = this->get_parameter("rotation_cycle_seconds").as_int();

        // レコードディレクトリが存在しない場合は作成
        if (!std::filesystem::exists(record_directory_)) {
            std::filesystem::create_directories(record_directory_);
        }

        // ログで操作説明を表示
        log_instructions();

        // カメラ初期化
        init_camera();

        // --- サブスクライバーの作成 (録画開始/停止をBoolで受け取る) ---
        recording_subscriber_ = this->create_subscription<std_msgs::msg::String>(
            "recording_status", // トピック名
            10,                 // キューサイズ
            std::bind(&MetavisionViewerNode::handle_recording_control, this, std::placeholders::_1)
        );

        // --- フレーム更新用のタイマー (表示処理のみ) ---
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(timer_interval_ms_),
            std::bind(&MetavisionViewerNode::timer_callback, this)
        );
    }

    ~MetavisionViewerNode() {
        // デストラクタで録画中であれば止める
        if (is_recording_) {
            stop_recording();
        }
        // カメラが動いていればストップ
        if (camera_.is_running()) {
            camera_.stop();
        }
    }

private:
    // --- ログメッセージを表示する ---
    void log_instructions() const {
        RCLCPP_INFO(this->get_logger(), "=== Metavision Viewer Node Instructions ===");
        RCLCPP_INFO(this->get_logger(), "Parameters:");
        RCLCPP_INFO(this->get_logger(), "  input_event_file: %s", input_event_file_.c_str());
        RCLCPP_INFO(this->get_logger(), "  bias_file: %s", bias_file_.c_str());
        RCLCPP_INFO(this->get_logger(), "  record_directory: %s", record_directory_.c_str());
        RCLCPP_INFO(this->get_logger(), "  frame_rate_ms: %d", timer_interval_ms_);
        RCLCPP_INFO(this->get_logger(), "  rotation_cycle_seconds: %d", rotation_cycle_seconds_);
        RCLCPP_INFO(this->get_logger(), "===========================================");
    }

    // --- カメラ初期化 ---
    void init_camera() {
        try {
            if (!input_event_file_.empty()) {
                RCLCPP_INFO(this->get_logger(), "Opening event file: %s", input_event_file_.c_str());
                camera_ = Metavision::Camera::from_file(input_event_file_);
            } else {
                RCLCPP_INFO(this->get_logger(), "Opening camera from first available device.");
                camera_ = Metavision::Camera::from_first_available();
            }

            if (!bias_file_.empty()) {
                RCLCPP_INFO(this->get_logger(), "Loading bias file: %s", bias_file_.c_str());
                camera_.biases().set_from_file(bias_file_);
            } else {
                RCLCPP_WARN(this->get_logger(), "No bias file specified. Using default biases.");
            }

            // CDFrameGenerator の初期化 (イベントから画像を生成)
            auto geometry = camera_.geometry();
            cd_frame_generator_ = std::make_unique<Metavision::CDFrameGenerator>(geometry.width(), geometry.height());
            cd_frame_generator_->set_display_accumulation_time_us(10000);

            // イベントコールバック登録
            camera_.cd().add_callback(
                [this](const Metavision::EventCD *begin, const Metavision::EventCD *end) {
                    std::lock_guard<std::mutex> lock(cd_frame_mutex_);
                    cd_frame_generator_->add_events(begin, end);
                }
            );

            // カメラスタート
            camera_.start();

            // フレーム生成スタート
            cd_frame_generator_->start(
                30, [this](Metavision::timestamp, const cv::Mat &frame) {
                    std::lock_guard<std::mutex> lock(cd_frame_mutex_);
                    frame.copyTo(cd_frame_);
                }
            );
        } catch (const Metavision::CameraException &e) {
            RCLCPP_ERROR(this->get_logger(), "Failed to initialize camera: %s", e.what());
            rclcpp::shutdown();
        }
    }

    // --- 表示用タイマーのコールバック ---
    void timer_callback() {
        std::lock_guard<std::mutex> lock(cd_frame_mutex_);
        if (!cd_frame_.empty()) {
            cv::imshow("CD Events", cd_frame_);
            cv::waitKey(1);
        }
    }

    // --- サブスクライバーから録画開始/停止を受け取る ---
    void handle_recording_control(const std_msgs::msg::String::SharedPtr msg) 
    {
        if (msg->data == "start") {
            if (!is_recording_) {
                start_recording();
            } else {
                RCLCPP_WARN(this->get_logger(), "Recording is already running.");
            }
        } else if (msg->data == "stop") {
            if (is_recording_) {
                stop_recording();
            } else {
                RCLCPP_WARN(this->get_logger(), "Recording is not currently running.");
            }
        } else if (msg->data == "switch") {
            if (is_recording_) {
                // ファイル切り替えを実行
                stop_recording();
                start_recording();
            } else {
                RCLCPP_WARN(this->get_logger(), "Cannot switch files because recording is not running.");
            }
        } else {
            RCLCPP_WARN(this->get_logger(), "Unknown command received: %s", msg->data.c_str());
        }
    }


    // --- 録画開始時の処理 ---
    void start_recording() {
        try {
            // 現在時刻からファイル名を生成
            auto now = std::chrono::system_clock::now();
            auto duration = now.time_since_epoch();
            auto microseconds = std::chrono::duration_cast<std::chrono::microseconds>(duration).count() % 1000000;
            std::time_t current_time = std::chrono::system_clock::to_time_t(now);
            std::tm *local_time = std::localtime(&current_time);

            char formatted_time[100];
            std::strftime(formatted_time, sizeof(formatted_time), "%Y%m%d_%H%M%S", local_time);

            std::ostringstream microseconds_stream;
            microseconds_stream << std::setw(6) << std::setfill('0') << microseconds;

            std::string directory_name = std::string(formatted_time) + "_" + microseconds_stream.str();
            std::string record_dir = record_directory_ + "/" + directory_name;

            // ディレクトリがなければ作成
            if (!std::filesystem::exists(record_dir)) {
                std::filesystem::create_directories(record_dir);
            }

            // .rawファイル名を作成して録画開始
            std::string record_file = record_dir + "/" + directory_name + ".raw";
            camera_.start_recording(record_file);
            RCLCPP_INFO(this->get_logger(), "Started recording to: %s", record_file.c_str());
            is_recording_ = true;

            // ---【変更点】録画開始と同時にローテーション用タイマーをスタート ---
            // すでにタイマーがなければ作成する (多重起動防止)
            if (!rotation_timer_) {
                rotation_timer_ = this->create_wall_timer(
                    std::chrono::seconds(rotation_cycle_seconds_),
                    std::bind(&MetavisionViewerNode::rotate_recording_file, this)
                );
            }
        } catch (const Metavision::CameraException &e) {
            RCLCPP_ERROR(this->get_logger(), "Failed to start recording: %s", e.what());
        }
    }

    // --- 録画停止時の処理 ---
    void stop_recording() {
        try {
            camera_.stop_recording();
            RCLCPP_INFO(this->get_logger(), "Stopped recording.");
            is_recording_ = false;

            // ---【変更点】タイマーを停止＆リセット (録画が終わったら必要ない) ---
            if (rotation_timer_) {
                rotation_timer_->cancel();
                rotation_timer_.reset();
            }
        } catch (const Metavision::CameraException &e) {
            RCLCPP_ERROR(this->get_logger(), "Failed to stop recording: %s", e.what());
        }
    }

    // ---【変更点】周期的にファイルを切り替えるコールバック ---
    void rotate_recording_file() {
        if (is_recording_) {
            // 現在の録画を停止して新しいファイルで再度録画開始
            stop_recording();
            start_recording();
        }
    }

    // --- メンバ変数 ---
    rclcpp::TimerBase::SharedPtr timer_;          // 表示用タイマー
    rclcpp::TimerBase::SharedPtr rotation_timer_; // 周期ファイル切り替え用タイマー

    int timer_interval_ms_;        // 表示フレーム更新のインターバル (ms)
    int rotation_cycle_seconds_;   // ファイル切り替え周期 (秒)

    std::string input_event_file_;
    std::string bias_file_;
    std::string record_directory_;

    bool is_recording_ = false;

    Metavision::Camera camera_;
    std::unique_ptr<Metavision::CDFrameGenerator> cd_frame_generator_;
    std::mutex cd_frame_mutex_;
    cv::Mat cd_frame_;

    // 録画開始/停止を受け取るサブスクライバー
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr recording_subscriber_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MetavisionViewerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
