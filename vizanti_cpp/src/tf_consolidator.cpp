#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "tf2_msgs/msg/tf_message.hpp"

class TfConsolidator:public rclcpp::Node{
	public:

		bool updated;
		std::map<std::string, geometry_msgs::msg::TransformStamped> transforms;
		std::map<std::string, rclcpp::Time> transform_timeout;

		rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr tf_sub;
		rclcpp::Publisher<tf2_msgs::msg::TFMessage>::SharedPtr tf_pub;
		rclcpp::TimerBase::SharedPtr clear_timer;

		TfConsolidator():Node("vizanti_tf_consolidator"),  updated(false){
			tf_sub = create_subscription<tf2_msgs::msg::TFMessage>("/tf", 10, std::bind(&TfConsolidator::tf_callback, this, std::placeholders::_1));
			tf_pub = create_publisher<tf2_msgs::msg::TFMessage>(
				"/vizanti/tf_consolidated",
				rclcpp::QoS(10)
					.durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL)
					.lifespan(rclcpp::Duration(0, 66000000))  // 66ms
			);
			clear_timer = create_wall_timer(std::chrono::seconds(5), std::bind(&TfConsolidator::clear_old_tfs, this));

			RCLCPP_INFO(get_logger(), "TF handler ready.");
		}

		/* void clear_old_tfs(){
			for (const auto &key : transforms){
				if (rclcpp::Clock().now().seconds() - transform_timeout[key.first].seconds() > 10.0){
					const auto parent = transforms[key.first].header.frame_id;
					transforms.erase(key.first);
					transform_timeout.erase(key.first);
					updated = true;
					RCLCPP_WARN(get_logger(), "Deleted old TF link: %s -> %s", parent.c_str(), key.first.c_str());
				}
			}
		} */

		void clear_old_tfs(){
			auto current_time = rclcpp::Clock().now();
			for (auto it = transforms.begin(); it != transforms.end(); /* no increment */) {
				const auto &key = it->first;
				if (current_time.seconds() - transform_timeout[key].seconds() > 10.0){
					const auto parent = it->second.header.frame_id;
					it = transforms.erase(it);
					transform_timeout.erase(key);
					updated = true;
					RCLCPP_WARN(get_logger(), "Removed old TF link: %s -> %s", parent.c_str(), key.c_str());
				} else {
					++it;
				}
			}
		}

		void publish(){
			if (!updated){
				return;
			}

			auto msg = tf2_msgs::msg::TFMessage();
			for (const auto &entry : transforms){
				msg.transforms.push_back(entry.second);
			}
			updated = false;
			tf_pub->publish(msg);
		}

		void tf_callback(const tf2_msgs::msg::TFMessage::SharedPtr msg){
			for (const auto &transform : msg->transforms){
				transforms[transform.child_frame_id] = transform;
				transform_timeout[transform.child_frame_id] = rclcpp::Clock().now();
			}
			updated = true;
		}
};

int main(int argc, char *argv[])
{
	rclcpp::init(argc, argv);
	auto odr = std::make_shared<TfConsolidator>();

	rclcpp::Rate rate(31);
	while (rclcpp::ok())
	{
		odr->publish();
		rclcpp::spin_some(odr);
		rate.sleep();
	}

	rclcpp::shutdown();
	return 0;
}