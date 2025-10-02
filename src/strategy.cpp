/*An example of the Strategy Design pattern in a ROS 2 context.
The Strategy Design pattern determines how an object behaves
The following example polymorphically changes functionality based on ROS params

A strategy pattern has the following classes
    - An abstract component class
    - Concrete component classes
    - A client class that contains a pointer to the abstract component class

A publisher takes a number and publishes it transformed with two different strategies
    - double the number (or)
    - squared of the number

This strategy / behaviour can be changed during runtime

To test:
    - ros2 run mtrx3760 strategy in one terminal (intended number)
    - ros2 topic echo numbers in another terminal (published number)
    - set / unset parameters through the following command in another terminal
        - ros2 param set /arithmetic_node number_strategy <double/squared>

By Khit Nay 2025
*/

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"

using StdInt32 = std_msgs::msg::Int32;

class NumberStrategy
{
    public:
        virtual StdInt32 transform(int input) = 0;
};

class DoubleStrategy : public NumberStrategy
{
    public:
        StdInt32 transform(int input) override;
};

class SquaredStrategy : public NumberStrategy
{
    public:
        StdInt32 transform(int input) override;
};

class Arithmetic : public rclcpp::Node
{
    public:
        Arithmetic();
    
    private:
        void setStrategy(std::string mode);
        void timerCallback();
        void paramCallback(const rclcpp::Parameter& param);

        int counter;
        
        std::string mode;
        std::unique_ptr<NumberStrategy> numberStrategy;
        rclcpp::TimerBase::SharedPtr timer;
        rclcpp::Publisher<StdInt32>::SharedPtr publisher;

        std::shared_ptr<rclcpp::ParameterEventHandler> paramEventHandler;
        rclcpp::ParameterCallbackHandle::SharedPtr paramCallbackHandle;

};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    std::shared_ptr<Arithmetic> node = std::make_shared<Arithmetic>();
    rclcpp::spin(node);
    rclcpp::shutdown();
}

StdInt32 DoubleStrategy::transform(int input)
{
    StdInt32 msg;
    msg.data = input * 2;
    return msg;
}

StdInt32 SquaredStrategy::transform(int input)
{
    StdInt32 msg;
    msg.data = input * input;
    return msg;
}

Arithmetic::Arithmetic()
    : Node("arithmetic_node"), counter(0)
{
    mode = declare_parameter<std::string>("number_strategy", "double");
    setStrategy(mode);

    paramEventHandler = std::make_shared<rclcpp::ParameterEventHandler>(this);
    paramCallbackHandle = paramEventHandler->add_parameter_callback("number_strategy",
        std::bind(&Arithmetic::paramCallback, this, std::placeholders::_1));

    publisher = create_publisher<StdInt32>("numbers", 10);
    timer = create_wall_timer(std::chrono::milliseconds(3000),
            std::bind(&Arithmetic::timerCallback, this));

}

void Arithmetic::setStrategy(std::string mode)
{
    if (mode == "double")
    {
        numberStrategy = std::make_unique<DoubleStrategy>();
    }
    else if (mode == "squared")
    {
        numberStrategy = std::make_unique<SquaredStrategy>();
    }
    else
    {
        rclcpp::shutdown();
    }
}

void Arithmetic::timerCallback()
{
    RCLCPP_INFO(get_logger(), "Using number %d", counter);
    StdInt32 msg = numberStrategy->transform(counter);
    publisher->publish(msg);
    counter++;
}

void Arithmetic::paramCallback(const rclcpp::Parameter& param)
{
    (void)param;
    mode = get_parameter("number_strategy").as_string();
    setStrategy(mode);
    RCLCPP_INFO(get_logger(), "Strategy changed");
}