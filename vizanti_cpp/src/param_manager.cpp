#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"

class ParamManager:public rclcpp::Node{
	public:

		rclcpp::TimerBase::SharedPtr timer;

		ParamManager():Node("vizanti_param_manager"){
			/* timer = create_wall_timer(std::chrono::seconds(20), std::bind(&ParamManager::update, this));
			RCLCPP_INFO(get_logger(), "Param manager handler ready.");

			auto parameters_client = rclcpp::SyncParametersClient(shared_from_this(), "/rviz");
			while (!parameters_client.wait_for_service(std::chrono::seconds(1))) {
				if (!rclcpp::ok()) {
					RCLCPP_ERROR(get_logger(), "Interrupted while waiting for the service. Exiting.");
					rclcpp::shutdown();
				}
				RCLCPP_INFO(get_logger(), "Service not available, waiting again...");
			}

			auto parameters = parameters_client.list_parameters({}, 1, true);

			// Print the list of parameters
			RCLCPP_INFO(get_logger(), "List of Parameters:");
			for (const auto &param : parameters.names) {
				RCLCPP_INFO(get_logger(), param);
			} */

		}

		void update(){
			/*RCLCPP_INFO(get_logger(), "get_node_names");
			auto node_names = get_node_names();

			for (const auto &pair : node_names) {
				RCLCPP_INFO(get_logger(), "Node Name: %s", pair.c_str());
			}*/
		}
};

int main(int argc, char *argv[])
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<ParamManager>());
	rclcpp::shutdown();
	return 0;
}

