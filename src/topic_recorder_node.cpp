#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <yaml-cpp/yaml.h>
#include <filesystem>
#include <string>
#include <map>

namespace fs = std::filesystem;

namespace topic_recorder
{

/**
 * @brief 话题配置
 */
struct TopicConfig {
    std::string name;        // 话题名称，如 "/image_raw"
    std::string type;        // "image"
    std::string topic_dir;   // 实际存储的目录
    std::string format_str;  // "mp4"/"avi"
    int fps;                 // 帧率
};

/**
 * @brief 将话题名中的斜杠转换成下划线
 */
static std::string sanitize_topic_name(const std::string & topic_name)
{
    std::string result = topic_name;
    for (auto & ch : result) {
        if (ch == '/') {
            ch = '_';
        }
    }
    while (!result.empty() && result.front() == '_') {
        result.erase(result.begin());
    }
    while (!result.empty() && result.back() == '_') {
        result.pop_back();
    }
    if (result.empty()) {
        result = "root";
    }
    return result;
}

class TopicRecorderNode : public rclcpp::Node
{
public:
    explicit TopicRecorderNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
    : Node("topic_recorder_node", options)
    {
        config_path_ = this->declare_parameter("config_path", std::string(""));
        RCLCPP_INFO(this->get_logger(), "TopicRecorderNode 启动");
        RCLCPP_INFO(this->get_logger(), "配置文件路径: %s", config_path_.c_str());

        if (!config_path_.empty()) {
            try {
                parse_config(config_path_);
            } catch (const std::exception & e) {
                RCLCPP_ERROR(this->get_logger(), "解析配置文件失败: %s", e.what());
                return;
            }
            setup_subscribers();
        } else {
            RCLCPP_ERROR(this->get_logger(), "未提供 config_path 参数");
        }
    }

private:
    std::string config_path_;
    std::string output_dir_;  
    std::vector<TopicConfig> topics_;
    std::map<std::string, cv::VideoWriter> video_writers_;
    std::vector<rclcpp::SubscriptionBase::SharedPtr> subscriptions_;

    void parse_config(const std::string & config_path)
    {
        YAML::Node root = YAML::LoadFile(config_path);
        auto recorder_node = root["topic_recorder"];
        if (!recorder_node) {
            throw std::runtime_error("未找到 topic_recorder 配置");
        }

        if (recorder_node["output_dir"]) {
            output_dir_ = recorder_node["output_dir"].as<std::string>();
        } else {
            output_dir_ = "./";
        }

        if (!recorder_node["topics"] || !recorder_node["topics"].IsSequence()) {
            throw std::runtime_error("topic_recorder.topics 缺失或类型错误");
        }

        for (auto topic_item : recorder_node["topics"]) {
            if (!topic_item["name"] || !topic_item["type"]) {
                RCLCPP_WARN(this->get_logger(), "忽略不完整的话题配置");
                continue;
            }

            if (topic_item["type"].as<std::string>() != "image") {
                continue;  // 跳过非图像话题
            }

            TopicConfig tc;
            tc.name = topic_item["name"].as<std::string>();
            tc.type = topic_item["type"].as<std::string>();
            tc.fps  = 30;  // 默认 30fps

            auto san_topic = sanitize_topic_name(tc.name);
            tc.topic_dir = output_dir_ + "/" + san_topic;

            if (topic_item["format"]) {
                tc.format_str = topic_item["format"].as<std::string>();
            }
            if (topic_item["fps"]) {
                tc.fps = topic_item["fps"].as<int>();
            }

            RCLCPP_INFO(
                this->get_logger(),
                "解析话题: '%s', 类型: '%s', 文件夹: '%s'",
                tc.name.c_str(),
                tc.type.c_str(),
                tc.topic_dir.c_str()
            );
            topics_.push_back(tc);
        }
    }

    void setup_subscribers()
    {
        for (auto & topic : topics_) {
            setup_image_subscriber(topic);
        }
    }

    void setup_image_subscriber(const TopicConfig & topic)
    {
        create_directory(topic.topic_dir);

        auto video_path = topic.topic_dir + "/recording_" +
                        std::to_string(std::time(nullptr));

        if (!topic.format_str.empty()) {
            video_path += "." + topic.format_str;
        } else {
            video_path += ".mp4";
        }

        auto qos = rclcpp::QoS(5)  // 保留最新的5帧
                    .best_effort()         // 允许丢包
                    .durability_volatile() // 不持久化
                    .reliability(rclcpp::ReliabilityPolicy::BestEffort);  // 不保证可靠性

        auto callback = [this, topic, video_path](const sensor_msgs::msg::Image::SharedPtr msg) {
            try {
                cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
                if (video_writers_.find(topic.name) == video_writers_.end()) {
                    int fourcc = cv::VideoWriter::fourcc('m','p','4','v');
                    if (topic.format_str == "avi") {
                        fourcc = cv::VideoWriter::fourcc('X','V','I','D');
                    }
                    video_writers_[topic.name] = cv::VideoWriter(
                        video_path,
                        fourcc,
                        topic.fps,
                        cv::Size(cv_ptr->image.cols, cv_ptr->image.rows)
                    );
                    if (!video_writers_[topic.name].isOpened()) {
                        RCLCPP_ERROR(this->get_logger(), "无法创建视频文件: %s", video_path.c_str());
                        return;
                    }
                    RCLCPP_INFO(this->get_logger(), "创建视频文件: %s", video_path.c_str());
                }
                video_writers_[topic.name].write(cv_ptr->image);
            } catch (const cv_bridge::Exception & e) {
                RCLCPP_ERROR(this->get_logger(), "cv_bridge 异常: %s", e.what());
            }
        };

        auto sub = create_subscription<sensor_msgs::msg::Image>(topic.name, qos, callback);
        subscriptions_.push_back(sub);
    }

    void create_directory(const std::string & path)
    {
        try {
            if (!fs::exists(path)) {
                fs::create_directories(path);
                RCLCPP_INFO(this->get_logger(), "创建目录: %s", path.c_str());
            }
        } catch (const std::exception & e) {
            RCLCPP_ERROR(this->get_logger(), "创建目录失败 %s: %s", path.c_str(), e.what());
            throw;
        }
    }
};

}  // namespace topic_recorder

RCLCPP_COMPONENTS_REGISTER_NODE(topic_recorder::TopicRecorderNode)