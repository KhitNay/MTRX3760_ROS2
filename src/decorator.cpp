#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>

class VelocityPublisher
{
    public:
        virtual void publish(double linearX) = 0;
};

class BasicVelocityPublisher : public VelocityPublisher
{
    public:
        BasicVelocityPublisher(rclcpp::Node* node)
        {
            publisher = node->create_publisher<geometry_msgs::msg::Twist>("linear_velocity", 10);
        }

        void publish(double linearX) override
        {
            geometry_msgs::msg::Twist msg;
            msg.linear.x = linearX;
            publisher->publish(msg);
        }

    private:
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher;
};

class VelocityPublisherDecorator : public VelocityPublisher
{
    public:
        VelocityPublisherDecorator(const std::shared_ptr<VelocityPublisher>& publisher)
            : publisher(publisher) {}

    protected:
        std::shared_ptr<VelocityPublisher> publisher;
};

class ForwardVelocityDecorator : public VelocityPublisherDecorator
{
    public:
        ForwardVelocityDecorator(const std::shared_ptr<VelocityPublisher>& publisher)
            : VelocityPublisherDecorator(publisher) {}

        void publish(double linearX) override
        {
            if (linearX >= 0)
            {
                publisher->publish(linearX);
            }
        }
};

class SafeVelocityDecorator : public VelocityPublisherDecorator
{
    public:
        SafeVelocityDecorator(const std::shared_ptr<VelocityPublisher>& publisher)
            : VelocityPublisherDecorator(publisher) {}

        void publish(double linearX) override
        {
            if (linearX > 3)
            {
                linearX = 3;
            }
            publisher->publish(linearX);
        }
};

class DecoratorNode : public rclcpp::Node
{
    public:
        DecoratorNode()
            : Node("decorator_node"), velocity(0), increment(1)
        {
            publisher = std::make_shared<BasicVelocityPublisher>(this);

            // Wrap with extra functionality as you wish.
            // publisher = std::make_shared<ForwardVelocityDecorator>(publisher);
            publisher = std::make_shared<SafeVelocityDecorator>(publisher);

            timer = create_wall_timer(std::chrono::seconds(1),
                    std::bind(&DecoratorNode::callback, this));
        }

    private:
        void callback()
        {
            // Simulate velocity goes from -5 to 5 and back down
            if (velocity == 5.0)
            {
                increment = -1;
            }
            else if (velocity == -5.0)
            {
                increment = 1;
            }

            velocity += increment;
            RCLCPP_INFO(get_logger(), "Intended velocity = %d", velocity);
            publisher->publish(velocity);
        }

        rclcpp::TimerBase::SharedPtr timer;
        std::shared_ptr<VelocityPublisher> publisher;
        double velocity;
        int increment;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DecoratorNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
