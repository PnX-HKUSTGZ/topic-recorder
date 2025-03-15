#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <rclcpp/serialization.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/bool.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <yaml-cpp/yaml.h>
#include <fstream>
#include <memory>
#include <string>
#include <vector>
#include <map>
#include <filesystem>

namespace fs = std::filesystem;

namespace topic_recorder
{

/**
 * @brief 数据变量描述（仅用于记录变量的名称与类型）
 */
struct DataVariable
{
  std::string name;
  std::string type;
};

/**
 * @brief 话题配置
 */
struct TopicConfig {
  std::string name;                // 话题名称，如 "/image_raw"
  std::string type;                // "image" 或 "data"
  std::string topic_dir;           // 实际存储的目录 = output_dir_ + "/" + 替换后的话题名
  std::string format_str;          // 对 "image" 来说可能是 "mp4/avi"；对 "data" 可不使用
  int fps;                         // 仅对图像话题有效
  std::vector<DataVariable> vars;  // 对 “data” 话题，要记录的变量列表
};

/**
 * @brief 用于将话题名中的斜杠转换成下划线，防止在创建目录或文件时出现多级目录问题
 */
static std::string sanitize_topic_name(const std::string & topic_name)
{
  std::string result = topic_name;
  for (auto & ch : result) {
    if (ch == '/') {
      ch = '_';
    }
  }
  // 去除可能首尾的下划线
  while (!result.empty() && result.front() == '_') {
    result.erase(result.begin());
  }
  while (!result.empty() && result.back() == '_') {
    result.pop_back();
  }
  if (result.empty()) {
    result = "root"; // 防止话题名只有斜杠导致空字符串
  }
  return result;
}

class TopicRecorderNode : public rclcpp::Node
{
public:
  explicit TopicRecorderNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("topic_recorder_node", options)
  {
    // 从 ROS 参数中读 config_path
    config_path_ = this->declare_parameter("config_path", std::string(""));
    RCLCPP_INFO(this->get_logger(), "TopicRecorderNode 启动");
    RCLCPP_INFO(this->get_logger(), "配置文件路径: %s", config_path_.c_str());

    // 如果传入了配置文件，则解析
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

  // 对图像话题：key=话题名，value=视频写入器
  std::map<std::string, cv::VideoWriter> video_writers_;
  // 对数据话题：key=话题名, value= <变量名, 对应ofstream>
  std::map<std::string, std::map<std::string, std::ofstream>> data_files_;

  std::vector<rclcpp::SubscriptionBase::SharedPtr> subscriptions_;

  /**
   * @brief 解析 YAML 文件，对应您给出的 topic_record_params.yaml 格式
   */
  void parse_config(const std::string & config_path)
  {
    YAML::Node root = YAML::LoadFile(config_path);
    auto recorder_node = root["topic_recorder"];
    if (!recorder_node) {
      throw std::runtime_error("未找到 topic_recorder 配置");
    }

    // 全局输出目录
    if (recorder_node["output_dir"]) {
      output_dir_ = recorder_node["output_dir"].as<std::string>();
    } else {
      output_dir_ = "./";
    }

    // 读取 topics 数组
    if (!recorder_node["topics"] || !recorder_node["topics"].IsSequence()) {
      throw std::runtime_error("topic_recorder.topics 缺失或类型错误");
    }

    for (auto topic_item : recorder_node["topics"]) {
      if (!topic_item["name"] || !topic_item["type"]) {
        RCLCPP_WARN(this->get_logger(), "忽略不完整的话题配置");
        continue;
      }

      TopicConfig tc;
      tc.name = topic_item["name"].as<std::string>();
      tc.type = topic_item["type"].as<std::string>();
      tc.fps  = 0;  // 默认为 0

      // 根据话题名创建其对应的文件夹路径
      // e.g. output_dir_/image_raw
      auto san_topic = sanitize_topic_name(tc.name);
      tc.topic_dir   = output_dir_ + "/" + san_topic;

      // 如果是图像话题：解析 format/fps
      if (tc.type == "image") {
        if (topic_item["format"]) {
          tc.format_str = topic_item["format"].as<std::string>(); // mp4, avi 等
        }
        if (topic_item["fps"]) {
          tc.fps = topic_item["fps"].as<int>();
        }
      }
      // 如果是数据话题：解析 format 下的变量信息
      else if (tc.type == "data") {
        if (topic_item["format"] && topic_item["format"].IsSequence()) {
          for (auto fmt_node : topic_item["format"]) {
            for (auto it = fmt_node.begin(); it != fmt_node.end(); ++it) {
              // it->first 例如 "variables[0]"
              const auto & varArray = it->second; 
              if (!varArray.IsSequence()) {
                continue;
              }
              for (auto var_item : varArray) {
                DataVariable dv;
                // 这里格式为:
                // - name: ...
                //   type: ...
                if (!var_item["name"] || !var_item["type"]) {
                  continue;
                }
                dv.name = var_item["name"].as<std::string>();
                dv.type = var_item["type"].as<std::string>();
                tc.vars.push_back(dv);
              }
            }
          }
        }
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

  /**
   * @brief 根据解析结果创建订阅
   */
  void setup_subscribers()
  {
    for (auto & topic : topics_) {
      if (topic.type == "image") {
        setup_image_subscriber(topic);
      } else if (topic.type == "data") {
        setup_data_subscriber(topic);
      } else {
        RCLCPP_WARN(this->get_logger(), "未知话题类型: %s", topic.type.c_str());
      }
    }
  }

  /**
   * @brief 设置图像类话题的订阅
   */
  void setup_image_subscriber(const TopicConfig & topic)
  {
    // 创建话题对应的目录
    create_directory(topic.topic_dir);

    // 用当前时间戳作为视频文件名
    auto video_path = topic.topic_dir + "/recording_" +
                      std::to_string(std::time(nullptr));

    // 如果 format_str 不为空，则加上后缀
    if (!topic.format_str.empty()) {
      video_path += "." + topic.format_str;
    } else {
      // 若为空，默认给一个 .mp4 后缀
      video_path += ".mp4";
    }

    // 订阅回调
    auto callback = [this, topic, video_path](const sensor_msgs::msg::Image::SharedPtr msg) {
      // 将 ROS 图像转换为 OpenCV
      try {
        cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
        if (video_writers_.find(topic.name) == video_writers_.end()) {
          // 初始化
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
        // 写入帧
        video_writers_[topic.name].write(cv_ptr->image);
      } catch (const cv_bridge::Exception & e) {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge 异常: %s", e.what());
      }
    };

    // 创建订阅
    auto sub = create_subscription<sensor_msgs::msg::Image>(topic.name, 10, callback);
    subscriptions_.push_back(sub);
  }

  /**
   * @brief 设置数据类话题的订阅
   * 将每个变量的名称作为单独的文件存储
   */
  void setup_data_subscriber(const TopicConfig & topic)
  {
    // 创建话题对应的目录
    create_directory(topic.topic_dir);

    auto callback = [this, topic](std::shared_ptr<rclcpp::SerializedMessage> msg) {
      // 记录 ROS 时间戳、消息大小等
      auto now = this->now();

      // 对于每个变量，在 (topic.topic_dir + "/" + var.name + ".txt") 打开文件并写入
      // 如需避免反复 open，可在 data_files_[topic.name][var.name] 找不到时创建
      for (auto & dv : topic.vars) {
        auto & file_map = data_files_[topic.name];  // map<varName, ofstream>
        if (file_map.find(dv.name) == file_map.end()) {
          // 文件不存在则创建
          std::string file_path = topic.topic_dir + "/" + dv.name + ".txt";
          std::ofstream ofs(file_path, std::ios::out | std::ios::app);
          if (!ofs.is_open()) {
            RCLCPP_ERROR(this->get_logger(), "无法创建文件: %s", file_path.c_str());
            continue;
          }
          file_map[dv.name] = std::move(ofs);
          RCLCPP_INFO(this->get_logger(), "创建变量文件: %s", file_path.c_str());
        }

        // 写入当前消息的简单信息
        auto & out_stream = file_map[dv.name];
        out_stream << "[" << now.seconds() << " s]  Raw message size: "
                   << msg->size() << " bytes\n";

        // ...existing code...
        // 在数据回调中，根据 dv.type 选择相应的反序列化逻辑
        out_stream << "[" << now.seconds() << " s]  Raw message size: "
        << msg->size() << " bytes\n";

        rclcpp::SerializationBase* base_serializer = nullptr;

        // 根据 dv.type 创建合适的序列化器，反序列化消息
        if (dv.type == "std_msgs/msg/String") {
          static rclcpp::Serialization<std_msgs::msg::String> serializer_str;
          base_serializer = &serializer_str;
        } else if (dv.type == "std_msgs/msg/Int32") {
          static rclcpp::Serialization<std_msgs::msg::Int32> serializer_int32;
          base_serializer = &serializer_int32;
        } else if (dv.type == "std_msgs/msg/Int32") {
          static rclcpp::Serialization<std_msgs::msg::Float32> serializer_float32;
          base_serializer = &serializer_float32;
        } else if (dv.type == "std_msgs/msg/Float64") {
          static rclcpp::Serialization<std_msgs::msg::Float64> serializer_float64;
          base_serializer = &serializer_float64;
        } else if (dv.type == "std_msgs/msg/Bool") {
          static rclcpp::Serialization<std_msgs::msg::Bool> serializer_bool;
          base_serializer = &serializer_bool;
        } else {
          out_stream << "未实现对类型 " << dv.type << " 的反序列化, 将仅记录原始大小\n";
        }

        // 若识别到可处理的类型，则进行反序列化并输出部分内容
        if (base_serializer) {
        // 这里我们需要一个临时的已知类型保存解析结果
        // 为了简化，可直接使用多态指针或在 if-else 内各自处理
        // 下面演示一个基于 if-else 的示例分支处理

        if (dv.type == "std_msgs/msg/String") {
        std_msgs::msg::String s;
        reinterpret_cast<rclcpp::Serialization<std_msgs::msg::String>*>(base_serializer)
        ->deserialize_message(msg.get(), &s);
        out_stream << "Variable: " << dv.name
            << " (type: String) => " << s.data << "\n";
        } else if (dv.type == "std_msgs/msg/Int32") {
        std_msgs::msg::Int32 val;
        reinterpret_cast<rclcpp::Serialization<std_msgs::msg::Int32>*>(base_serializer)
        ->deserialize_message(msg.get(), &val);
        out_stream << "Variable: " << dv.name
            << " (type: Int32) => " << val.data << "\n";
        } else if (dv.type == "std_msgs/msg/Float64") {
        std_msgs::msg::Float64 val;
        reinterpret_cast<rclcpp::Serialization<std_msgs::msg::Float64>*>(base_serializer)
        ->deserialize_message(msg.get(), &val);
        out_stream << "Variable: " << dv.name
            << " (type: Float64) => " << val.data << "\n";
        } else if (dv.type == "std_msgs/msg/Bool") {
        std_msgs::msg::Bool val;
        reinterpret_cast<rclcpp::Serialization<std_msgs::msg::Bool>*>(base_serializer)
        ->deserialize_message(msg.get(), &val);
        out_stream << "Variable: " << dv.name
            << " (type: Bool) => " << std::boolalpha << val.data << "\n";
        }
        }

        out_stream << "----------------------------------------\n";
        // ...existing code...
      }

      // 如果没有配置任何 vars，则可以选择写入一个默认文件
      if (topic.vars.empty()) {
        // 使用一个名字固定的文件
        auto & file_map = data_files_[topic.name];
        static const std::string default_var = "_no_vars_";
        if (file_map.find(default_var) == file_map.end()) {
          std::string file_path = topic.topic_dir + "/no_vars.txt";
          std::ofstream ofs(file_path, std::ios::out | std::ios::app);
          if (!ofs.is_open()) {
            RCLCPP_ERROR(this->get_logger(), "无法创建文件: %s", file_path.c_str());
            return;
          }
          file_map[default_var] = std::move(ofs);
          RCLCPP_INFO(this->get_logger(), "创建无变量文件: %s", file_path.c_str());
        }
        auto & out_stream = file_map[default_var];
        out_stream << "[" << now.seconds() << " s] Raw message size: "
                   << msg->size() << " bytes\n";
        out_stream << "----------------------------------------\n";
      }
    };

    // create_generic_subscription 让我们可以随意订阅任意消息类型
    auto sub = create_generic_subscription(topic.name, "std_msgs/msg/String", 10, callback);
    subscriptions_.push_back(sub);
  }

  /**
   * @brief 辅助函数，用于创建目录
   */
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