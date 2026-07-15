#include <gazebo/gazebo.hh>
#include <gazebo/gui/GuiPlugin.hh>

#include <rclcpp/rclcpp.hpp>

#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

#include <QApplication>
#include <QEvent>
#include <QKeyEvent>
#include <QLabel>
#include <QVBoxLayout>

#include <algorithm>
#include <memory>
#include <string>

namespace gazebo
{

class FishbotKeyboardGUI : public GUIPlugin
{
public:
    FishbotKeyboardGUI()
        : GUIPlugin()
    {
        /*
         * gzclient normally does not start ROS 2 itself.
         * Initialise ROS only if another GUI plugin has not
         * already done so.
         */
        if (!rclcpp::ok())
        {
            int argc = 0;
            char **argv = nullptr;

            rclcpp::init(argc, argv);
            owns_ros_context_ = true;
        }

        node_ = std::make_shared<rclcpp::Node>(
            "fishbot_gazebo_keyboard");

        speed_publisher_ =
            node_->create_publisher<std_msgs::msg::Float64>(
                "/fish_speed",
                10);

        turn_publisher_ =
            node_->create_publisher<std_msgs::msg::Float64>(
                "/fish_turn",
                10);

        left_fin_publisher_ =
            node_->create_publisher<
                std_msgs::msg::Float64MultiArray>(
                "/left_fin_controller/commands",
                10);

        right_fin_publisher_ =
            node_->create_publisher<
                std_msgs::msg::Float64MultiArray>(
                "/right_fin_controller/commands",
                10);

        /*
         * Install this class as a keyboard-event filter for
         * the complete Gazebo GUI application.
         */
        if (QApplication::instance())
        {
            QApplication::instance()->installEventFilter(
                this);
        }

        CreateControlPanel();

        gzmsg
            << "Fishbot Gazebo keyboard plugin loaded\n";

        RCLCPP_INFO(
            node_->get_logger(),
            "Click the Gazebo window, then use "
            "W/A/S/D/X/I/K/O/SPACE");

        /*
         * Publish initial state. Pressing a key will publish
         * again if the subscribers are not ready yet.
         */
        PublishAllCommands();
    }

    ~FishbotKeyboardGUI() override
    {
        if (QApplication::instance())
        {
            QApplication::instance()->removeEventFilter(
                this);
        }

        /*
         * Do not shut down ROS if another plugin originally
         * initialised it.
         */
        if (owns_ros_context_ && rclcpp::ok())
        {
            rclcpp::shutdown();
        }
    }

protected:
    bool eventFilter(
        QObject *object,
        QEvent *event) override
    {
        /*
         * Ignore keyboard events when Gazebo is not the active
         * application window.
         */
        if (!QApplication::activeWindow())
        {
            return GUIPlugin::eventFilter(
                object,
                event);
        }

        if (event->type() != QEvent::KeyPress)
        {
            return GUIPlugin::eventFilter(
                object,
                event);
        }

        auto *key_event =
            static_cast<QKeyEvent *>(event);

        /*
         * Ignore automatic key repetition so one long press does
         * not rapidly change the speed.
         */
        if (key_event->isAutoRepeat())
        {
            return true;
        }

        bool handled = true;

        switch (key_event->key())
        {
            case Qt::Key_W:
                IncreaseSpeed();
                break;

            case Qt::Key_S:
                DecreaseSpeed();
                break;

            case Qt::Key_A:
                TurnLeft();
                break;

            case Qt::Key_D:
                TurnRight();
                break;

            case Qt::Key_X:
                SwimStraight();
                break;

            case Qt::Key_I:
                PitchUp();
                break;

            case Qt::Key_K:
                PitchDown();
                break;

            case Qt::Key_O:
                NeutralFins();
                break;

            case Qt::Key_Space:
                StopFish();
                break;

            default:
                handled = false;
                break;
        }

        if (handled)
        {
            UpdateStatusLabel();

            /*
             * Return true so Gazebo does not process the same
             * control key for another shortcut.
             */
            return true;
        }

        return GUIPlugin::eventFilter(
            object,
            event);
    }

private:
    void CreateControlPanel()
    {
        /*
         * This creates a small status panel inside Gazebo.
         */
        auto *layout = new QVBoxLayout();

        title_label_ =
            new QLabel("Fishbot Keyboard Control");

        instruction_label_ = new QLabel(
            "W/S: Speed\n"
            "A/D: Turn\n"
            "X: Straight\n"
            "I/K: Pitch\n"
            "O: Neutral fins\n"
            "Space: Stop");

        status_label_ = new QLabel();

        title_label_->setStyleSheet(
            "font-weight: bold; "
            "font-size: 14px;");

        instruction_label_->setStyleSheet(
            "font-size: 12px;");

        status_label_->setStyleSheet(
            "font-size: 12px; "
            "color: #80dfff;");

        layout->addWidget(title_label_);
        layout->addWidget(instruction_label_);
        layout->addWidget(status_label_);

        setLayout(layout);

        move(10, 100);
        resize(190, 220);

        setStyleSheet(
            "background-color: "
            "rgba(30, 30, 30, 210); "
            "color: white; "
            "border: 1px solid #666;");

        UpdateStatusLabel();
    }

    void IncreaseSpeed()
    {
        speed_ = std::min(
            maximum_speed_,
            speed_ + speed_step_);

        PublishSpeed();

        RCLCPP_INFO(
            node_->get_logger(),
            "Swimming frequency: %.2f Hz",
            speed_);
    }

    void DecreaseSpeed()
    {
        speed_ = std::max(
            0.0,
            speed_ - speed_step_);

        PublishSpeed();

        RCLCPP_INFO(
            node_->get_logger(),
            "Swimming frequency: %.2f Hz",
            speed_);
    }

    void TurnLeft()
    {
        /*
         * Negative tail bias follows the steering convention
         * used in the current thrust plugin.
         */
        turn_bias_ = -maximum_turn_bias_;

        PublishTurn();

        RCLCPP_INFO(
            node_->get_logger(),
            "Turning left");
    }

    void TurnRight()
    {
        turn_bias_ = maximum_turn_bias_;

        PublishTurn();

        RCLCPP_INFO(
            node_->get_logger(),
            "Turning right");
    }

    void SwimStraight()
    {
        turn_bias_ = 0.0;

        PublishTurn();

        RCLCPP_INFO(
            node_->get_logger(),
            "Tail bias removed");
    }

    void PitchUp()
    {
        pitch_angle_ = pitch_fin_angle_;

        PublishFins();

        RCLCPP_INFO(
            node_->get_logger(),
            "Pitch-up command");
    }

    void PitchDown()
    {
        pitch_angle_ = -pitch_fin_angle_;

        PublishFins();

        RCLCPP_INFO(
            node_->get_logger(),
            "Pitch-down command");
    }

    void NeutralFins()
    {
        pitch_angle_ = 0.0;

        PublishFins();

        RCLCPP_INFO(
            node_->get_logger(),
            "Pectoral fins neutral");
    }

    void StopFish()
    {
        speed_ = 0.0;
        turn_bias_ = 0.0;
        pitch_angle_ = 0.0;

        PublishAllCommands();

        RCLCPP_INFO(
            node_->get_logger(),
            "Fish stopped");
    }

    void PublishSpeed()
    {
        std_msgs::msg::Float64 message;
        message.data = speed_;

        speed_publisher_->publish(message);
    }

    void PublishTurn()
    {
        std_msgs::msg::Float64 message;
        message.data = turn_bias_;

        turn_publisher_->publish(message);
    }

    void PublishFins()
    {
        std_msgs::msg::Float64MultiArray left_message;
        left_message.data = {pitch_angle_};

        std_msgs::msg::Float64MultiArray right_message;
        right_message.data = {pitch_angle_};

        left_fin_publisher_->publish(
            left_message);

        right_fin_publisher_->publish(
            right_message);
    }

    void PublishAllCommands()
    {
        PublishSpeed();
        PublishTurn();
        PublishFins();
    }

    void UpdateStatusLabel()
    {
        if (!status_label_)
            return;

        std::string direction = "Straight";

        if (turn_bias_ < 0.0)
            direction = "Left";
        else if (turn_bias_ > 0.0)
            direction = "Right";

        std::string pitch = "Neutral";

        if (pitch_angle_ > 0.0)
            pitch = "Up";
        else if (pitch_angle_ < 0.0)
            pitch = "Down";

        const QString status =
            QString(
                "Speed: %1 Hz\n"
                "Steering: %2\n"
                "Pitch: %3")
                .arg(speed_, 0, 'f', 1)
                .arg(
                    QString::fromStdString(
                        direction))
                .arg(
                    QString::fromStdString(
                        pitch));

        status_label_->setText(status);
    }

private:
    bool owns_ros_context_ = false;

    rclcpp::Node::SharedPtr node_;

    rclcpp::Publisher<
        std_msgs::msg::Float64>::SharedPtr
        speed_publisher_;

    rclcpp::Publisher<
        std_msgs::msg::Float64>::SharedPtr
        turn_publisher_;

    rclcpp::Publisher<
        std_msgs::msg::Float64MultiArray>::SharedPtr
        left_fin_publisher_;

    rclcpp::Publisher<
        std_msgs::msg::Float64MultiArray>::SharedPtr
        right_fin_publisher_;

    QLabel *title_label_ = nullptr;
    QLabel *instruction_label_ = nullptr;
    QLabel *status_label_ = nullptr;

    double speed_ = 1.0;
    double turn_bias_ = 0.0;
    double pitch_angle_ = 0.0;

    double maximum_speed_ = 1.2;
    double speed_step_ = 0.1;

    double maximum_turn_bias_ = 0.18;
    double pitch_fin_angle_ = 0.20;
};

GZ_REGISTER_GUI_PLUGIN(FishbotKeyboardGUI)

}