#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point.hpp"
#include <fstream>
#include <sstream>


class CsvPublisher : public rclcpp::Node
{
public:
   CsvPublisher() : Node("csv_publisher")
   {
       publisher_ = this->create_publisher<geometry_msgs::msg::Point>("/position", 10);
       timer_ = this->create_wall_timer(
           std::chrono::milliseconds(500), std::bind(&CsvPublisher::timer_callback, this));
       file_.open("vallelunga.csv");


       if (!file_.is_open())
       {
           RCLCPP_ERROR(this->get_logger(), "Errore nell'aprire il file CSV");
       }
       else
       {
           std::string line;
           std::getline(file_, line);
       }
   }


private:
   void timer_callback()
   {
       if (!file_.eof())
       {
           std::string line;
           std::getline(file_, line);
           std::stringstream ss(line);
           std::string timestamp, x, y, z;
           std::getline(ss, timestamp, ',');
           std::getline(ss, x, ',');
           std::getline(ss, y, ',');
           std::getline(ss, z, ',');


           geometry_msgs::msg::Point point;
           point.x = std::stod(x);
           point.y = std::stod(y);
           point.z = std::stod(z);
           publisher_->publish(point);
       }
       else
       {
           RCLCPP_INFO(this->get_logger(), "Fine CSV");
           rclcpp::shutdown();
       }
   }


   rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr publisher_;
   rclcpp::TimerBase::SharedPtr timer_;
   std::ifstream file_;
};


int main(int argc, char **argv)
{
   rclcpp::init(argc, argv);
   rclcpp::spin(std::make_shared<CsvPublisher>());
   rclcpp::shutdown();
   return 0;
}