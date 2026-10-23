// Store complete ROS CDR buffers. No message decoding, field selection or resampling.
#include <rclcpp/rclcpp.hpp>
#include <rosbag2_cpp/writer.hpp>
#include <rosbag2_transport/qos.hpp>
#include <yaml-cpp/yaml.h>

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <csignal>
#include <deque>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <map>
#include <mutex>
#include <set>
#include <sys/resource.h>
#include <thread>

namespace fs = std::filesystem;
static volatile std::sig_atomic_t stop_requested = 0;
static void stop_signal(int) {stop_requested = 1;}
static int64_t monotonic_ns()
{
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::steady_clock::now().time_since_epoch()).count();
}

class RawRecorder : public rclcpp::Node
{
  struct Topic {
    std::string type, offered_qos, requested_qos;
    uint64_t received = 0, written = 0;
    bool lost_events_supported = true;
  };
  struct Item {
    std::string topic, type, qos;
    std::shared_ptr<rclcpp::SerializedMessage> data;
    int64_t ros_ns = 0, monotonic = 0;
    uint64_t sequence = 0;
  };
  std::string directory_;
  std::vector<std::string> requested_;
  std::map<std::string, Topic> topics_;
  std::map<std::string, rclcpp::GenericSubscription::SharedPtr> subscriptions_;
  std::deque<Item> queue_;
  std::mutex mutex_;
  std::condition_variable ready_;
  std::thread worker_;
  bool stopping_ = false;
  std::atomic<bool> failed_{false};
  uint64_t capacity_, bytes_limit_, queued_bytes_ = 0, peak_bytes_ = 0, peak_count_ = 0;
  uint64_t overflow_ = 0, dds_lost_ = 0, incompatible_ = 0, received_ = 0, written_ = 0;
  std::vector<std::string> errors_;
  rosbag2_cpp::Writer writer_;
  rclcpp::TimerBase::SharedPtr discovery_, health_;
  const int64_t started_ = monotonic_ns();

  void error(const std::string & text)
  {
    {std::lock_guard<std::mutex> lock(mutex_); errors_.push_back(text);}
    failed_ = true;
    RCLCPP_ERROR(get_logger(), "%s", text.c_str());
  }

  void receive(const std::string & topic, std::shared_ptr<rclcpp::SerializedMessage> data)
  {
    const auto ros_ns = now().nanoseconds();
    const auto mono = monotonic_ns();
    const auto size = data->size();
    bool overflow = false;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      auto & state = topics_.at(topic);
      const auto seq = ++state.received;
      ++received_;
      if (queue_.size() >= capacity_ || size > bytes_limit_ - queued_bytes_) {
        ++overflow_;
        overflow = true;
      } else {
        queue_.push_back({topic, state.type, "", std::move(data), ros_ns, mono, seq});
        queued_bytes_ += size;
        peak_bytes_ = std::max(peak_bytes_, queued_bytes_);
        peak_count_ = std::max<uint64_t>(peak_count_, queue_.size());
      }
    }
    if (overflow) {
      error("Storage queue capacity exceeded; recording is incomplete (no overwrite permitted)");
    }
    ready_.notify_one();
  }

  void write_loop()
  {
    try {
      std::ofstream receipts(directory_ + "/receipts.csv");
      receipts.exceptions(std::ios::badbit | std::ios::failbit);
      // bag_message_index is global insertion order in this unsplit, uncached bag.
      receipts << "bag_message_index,topic,topic_sequence,received_ros_ns,received_monotonic_ns,serialized_bytes\n";
      while (true) {
        Item item;
        {
          std::unique_lock<std::mutex> lock(mutex_);
          ready_.wait(lock, [this] {return stopping_ || !queue_.empty();});
          if (queue_.empty()) {break;}
          item = std::move(queue_.front());
          queue_.pop_front();
          if (item.data) {queued_bytes_ -= item.data->size();}
        }
        if (!item.data) {
          writer_.create_topic({item.topic, item.type, "cdr", item.qos});
          continue;
        }
        const auto bytes = item.data->size();
        // rosbag2 transfers ownership of the original serialized buffer; no conversion.
        writer_.write(item.data, item.topic, item.type, rclcpp::Time(item.ros_ns));
        receipts << written_ + 1 << ',' << item.topic << ',' << item.sequence << ','
                 << item.ros_ns << ',' << item.monotonic << ',' << bytes << '\n';
        {
          std::lock_guard<std::mutex> lock(mutex_);
          ++written_;
          ++topics_.at(item.topic).written;
        }
      }
      receipts.flush();
      receipts.close();
    } catch (const std::exception & e) {
      error(std::string("Writer failed: ") + e.what());
    }
    try {writer_.close();} catch (const std::exception & e) {
      error(std::string("Bag close failed: ") + e.what());
    }
  }

public:
  RawRecorder() : Node("vehicle_recorder", rclcpp::NodeOptions().enable_rosout(false))
  {
    directory_ = declare_parameter<std::string>("session_directory", "");
    requested_ = declare_parameter<std::vector<std::string>>("topics", std::vector<std::string>{});
    const auto path = declare_parameter<std::string>("reference_path_topic", "/control/reference_path");
    if (!path.empty()) {requested_.push_back(path);}
    std::set<std::string> unique(requested_.begin(), requested_.end());
    requested_.assign(unique.begin(), unique.end());
    const auto capacity = declare_parameter<int64_t>("record_queue_size", 4096);
    const auto byte_limit = declare_parameter<int64_t>("record_queue_bytes", 268435456);
    const auto depth = declare_parameter<int64_t>("subscription_depth", 4096);
    if (directory_.empty() || capacity <= 0 || byte_limit <= 0 || depth <= 0) {
      throw std::invalid_argument("session_directory and positive queue/depth limits are required");
    }
    capacity_ = static_cast<uint64_t>(capacity);
    bytes_limit_ = static_cast<uint64_t>(byte_limit);
    rosbag2_storage::StorageOptions storage;
    storage.uri = directory_ + "/bag";
    storage.storage_id = "sqlite3";
    storage.storage_preset_profile = "resilient";
    // Our queue is explicit; disable rosbag's separate drop-on-overflow cache.
    storage.max_cache_size = 0;
    writer_.open(storage, {"cdr", "cdr"});
    discovery_ = create_wall_timer(std::chrono::milliseconds(500), [this, depth] {
      discover(static_cast<size_t>(depth));
    });
    health_ = create_wall_timer(std::chrono::seconds(1), [this] {report(false);});
    report(false);
    worker_ = std::thread([this] {write_loop();});
  }

  void discover(size_t depth)
  {
    const auto graph = get_topic_names_and_types();
    for (const auto & name : requested_) {
      auto found = graph.find(name);
      if (found == graph.end()) {continue;}
      if (found->second.size() != 1) {
        error("Ambiguous message types for " + name);
        continue;
      }
      const auto type = found->second.front();
      if (subscriptions_.count(name)) {
        if (topics_.at(name).type != type) {error("Message type changed for " + name);}
        continue;
      }
      const auto endpoints = get_publishers_info_by_topic(name);
      if (endpoints.empty()) {continue;}
      auto qos = rosbag2_transport::Rosbag2QoS::adapt_request_to_offers(name, endpoints);
      qos.keep_last(depth);
      std::vector<rosbag2_transport::Rosbag2QoS> offers;
      for (const auto & ep : endpoints) {offers.emplace_back(ep.qos_profile());}
      YAML::Node yaml_offers(offers);
      YAML::Node yaml_request(qos);
      {
        std::lock_guard<std::mutex> lock(mutex_);
        topics_[name] = {type, YAML::Dump(yaml_offers), YAML::Dump(yaml_request)};
        queue_.push_back({name, type, YAML::Dump(yaml_offers), nullptr});
      }
      ready_.notify_one();
      rclcpp::SubscriptionOptions options;
      options.event_callbacks.message_lost_callback = [this, name](rclcpp::QOSMessageLostInfo & info) {
        {std::lock_guard<std::mutex> lock(mutex_); dds_lost_ += info.total_count_change;}
        error("DDS reported message loss on " + name);
      };
      options.event_callbacks.incompatible_qos_callback = [this, name](rclcpp::QOSRequestedIncompatibleQoSInfo & info) {
        {std::lock_guard<std::mutex> lock(mutex_); incompatible_ += info.total_count_change;}
        error("Incompatible publisher QoS on " + name);
      };
      try {
        try {
          subscriptions_[name] = create_generic_subscription(name, type, qos,
            [this, name](std::shared_ptr<rclcpp::SerializedMessage> m) {receive(name, std::move(m));}, options);
        } catch (const rclcpp::UnsupportedEventTypeException &) {
          options.event_callbacks = {};
          {std::lock_guard<std::mutex> lock(mutex_); topics_.at(name).lost_events_supported = false;}
          subscriptions_[name] = create_generic_subscription(name, type, qos,
            [this, name](std::shared_ptr<rclcpp::SerializedMessage> m) {receive(name, std::move(m));}, options);
        }
        RCLCPP_INFO(get_logger(), "Recording serialized %s [%s]", name.c_str(), type.c_str());
      } catch (const std::exception & e) {error("Subscription failed for " + name + ": " + e.what());}
    }
  }

  void report(bool closed)
  {
    try {
      YAML::Node n;
      struct rusage usage{};
      getrusage(RUSAGE_SELF, &usage);
      {
        std::lock_guard<std::mutex> lock(mutex_);
        n["closed"] = closed;
        n["complete"] = closed && !failed_ && queue_.empty() && received_ == written_;
        n["received"] = received_; n["written"] = written_;
        n["queue_overflow"] = overflow_; n["dds_reported_lost"] = dds_lost_;
        n["incompatible_qos"] = incompatible_;
        n["pending"] = queue_.size(); n["pending_bytes"] = queued_bytes_;
        n["peak_queue_bytes"] = peak_bytes_; n["peak_queue_messages"] = peak_count_;
        n["errors"] = errors_;
        n["requested_topics"] = requested_;
        n["topics"] = YAML::Node(YAML::NodeType::Map);
        for (const auto & pair : topics_) {
          auto t = n["topics"][pair.first];
          t["type"] = pair.second.type;
          t["received"] = pair.second.received; t["written"] = pair.second.written;
          t["offered_qos"] = YAML::Load(pair.second.offered_qos);
          t["requested_qos"] = YAML::Load(pair.second.requested_qos);
          t["dds_lost_events_supported"] = pair.second.lost_events_supported;
        }
      }
      n["elapsed_s"] = (monotonic_ns() - started_) / 1e9;
      n["cpu_s"] = usage.ru_utime.tv_sec + usage.ru_utime.tv_usec / 1e6 +
        usage.ru_stime.tv_sec + usage.ru_stime.tv_usec / 1e6;
      n["max_rss_kib"] = usage.ru_maxrss;
      std::ofstream f(directory_ + "/backend.yaml.tmp");
      f.exceptions(std::ios::badbit | std::ios::failbit);
      f << n << '\n';
      f.close();
      fs::rename(directory_ + "/backend.yaml.tmp", directory_ + "/backend.yaml");
    } catch (const std::exception & e) {
      error(std::string("Status metadata failed: ") + e.what());
    }
  }

  bool failed() const {return failed_;}

  bool finish()
  {
    discovery_.reset(); health_.reset(); subscriptions_.clear();
    {
      std::lock_guard<std::mutex> lock(mutex_);
      stopping_ = true;
    }
    ready_.notify_one();
    if (worker_.joinable()) {worker_.join();}
    report(true);
    return !failed_;
  }

  ~RawRecorder() override
  {
    if (worker_.joinable()) {finish();}
  }
};

int main(int argc, char ** argv)
{
  std::signal(SIGINT, stop_signal); std::signal(SIGTERM, stop_signal);
  int result = 0;
  try {
    rclcpp::init(argc, argv, rclcpp::InitOptions(), rclcpp::SignalHandlerOptions::None);
    auto node = std::make_shared<RawRecorder>();
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    while (rclcpp::ok() && !stop_requested && !node->failed()) {
      executor.spin_once(std::chrono::milliseconds(100));
    }
    if (!node->finish()) {result = 1;}
  } catch (const std::exception & e) {
    std::cerr << "Recorder backend failed: " << e.what() << std::endl;
    result = 1;
  }
  if (rclcpp::ok()) {rclcpp::shutdown();}
  return result;
}
