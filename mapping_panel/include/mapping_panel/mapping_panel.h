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
#include <QSlider>

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
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/float32.hpp"

//@ for mkdir
#include <filesystem>
#include <iostream>
#include <fstream>
#include <string>
#include <ctime>

namespace mapping_panel
{
  class MappingPanel : public rviz_common::Panel // QMainWindow
{

  Q_OBJECT
  
  public:

    explicit MappingPanel(QWidget *parent = nullptr);

    void onInitialize() override;

  private:

    QPushButton *pause_btn_;
    QPushButton *resume_btn_;

    QSlider* icp_score_slider_;
    QLabel* icp_score_value_;

    QSlider* history_keyframe_search_radius_slider_;
    QLabel* history_keyframe_search_radius_value_;

    QSlider* skip_frame_slider_;
    QLabel* skip_frame_value_;

    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pause_resume_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr icp_score_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr history_keyframe_search_radius_pub_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr skip_frame_pub_;

    rclcpp::Clock::SharedPtr clock_;
    
  // Here we declare some internal slots.
  protected Q_SLOTS:
    void pause();
    void resume();
    void setICPScoreValue();
    void setHistoryKeyframeSearchRadiusValue();
    void setSkipFrameValue();
};

} // namespace map_editor_panel
