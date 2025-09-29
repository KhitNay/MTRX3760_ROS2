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
        StdInt32 transform(int input) override
        {
            StdInt32 msg;
            msg.data = input * 2;
            return msg;
        }
};

class SquaredStrategy : public NumberStrategy
{
    public:
        StdInt32 transform(int input) override
        {
            StdInt32 msg;
            msg.data = input * input;
            return msg;
        }
};

class Arithmetic : public rclcpp::Node
{
    public:
        Arithmetic()
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
    private:

        void setStrategy(std::string mode)
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

        void timerCallback()
        {
            RCLCPP_INFO(get_logger(), "Using number %d", counter);
            StdInt32 msg = numberStrategy->transform(counter);
            publisher->publish(msg);
            counter++;
        }

        void paramCallback(const rclcpp::Parameter& param)
        {
            (void)param;
            mode = get_parameter("number_strategy").as_string();
            setStrategy(mode);
            RCLCPP_INFO(get_logger(), "Strategy changed");
        }

        int counter;
        rclcpp::TimerBase::SharedPtr timer;
        std::string mode;
        std::unique_ptr<NumberStrategy> numberStrategy;
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