#ifndef PLUGIN_INTERFACE_H
#define PLUGIN_INTERFACE_H


#include <rclcpp/rclcpp.hpp>
#include <example_plugin_manager/common_handlers.h>


namespace example_plugin_manager
{

class Plugin : public rclcpp::Node
{
    public:

        virtual void initialize(std::shared_ptr<rclcpp::Node> parent_node_, const std::string& name, const std::string& name_space,
                               std::shared_ptr<example_plugin_manager::CommonHandlers_t> common_handlers) = 0;

        virtual bool activate(const int& some_number) = 0;

        virtual void deactivate(void) = 0;

        virtual const std::optional<double> update() = 0;

        virtual ~Plugin() = default;
};
    
}

#endif
