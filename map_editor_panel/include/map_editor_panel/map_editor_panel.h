#include <QMainWindow>
#include "rviz_common/panel.hpp"

#include <QVBoxLayout>
#include <QLabel>
#include <QPushButton>
#include <QFileDialog>
#include <QRadioButton>
#include <QGroupBox>
#include <QFormLayout>
#include <QMessageBox>

#include "rclcpp/rclcpp.hpp"
#include <rviz_common/display_context.hpp>
#include "rviz_common/tool.hpp"
#include "std_msgs/msg/string.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/qos.hpp"

// PCL
#include "pcl/common/transforms.h"
#include <pcl/common/geometry.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <Eigen/Dense>
#include <pcl_conversions/pcl_conversions.h>

// ROS msg
#include "sensor_msgs/msg/point_cloud2.hpp"

//@ for mkdir
#include <filesystem>
#include <iostream>
#include <fstream>
#include <string>
#include <ctime>

namespace map_editor_panel
{
  class MapEditorPanel : public rviz_common::Panel // QMainWindow
{

  inline std::string currentDateTime() {
    std::time_t t = std::time(nullptr);
    std::tm* now = std::localtime(&t);

    char buffer[128];
    strftime(buffer, sizeof(buffer), "%Y_%m_%d_%H_%M_%S", now);
    return buffer;
  }

  Q_OBJECT
  
  public:

    explicit MapEditorPanel(QWidget *parent = nullptr);

    void onInitialize() override;

  private:

    QPushButton *save_dir_btn_;
    QPushButton *clear_selection_btn_;
    QPushButton *last_step_btn_;

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr operation_command_pub_;
    rclcpp::Clock::SharedPtr clock_;
    
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_selected_pc_;
    void selectedPCCb(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

    pcl::PointCloud<pcl::PointXYZ>::Ptr sub_pc_;

  // Here we declare some internal slots.
  protected Q_SLOTS:
    void saveSelectedPC();
    void clearSelection();
    void lastStep();
};

} // namespace map_editor_panel
