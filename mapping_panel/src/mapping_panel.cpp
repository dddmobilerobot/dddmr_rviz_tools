#include "mapping_panel/mapping_panel.h"

#include "rviz_common/properties/property_tree_widget.hpp"
#include "rviz_common/interaction/selection_manager.hpp"
#include "rviz_common/visualization_manager.hpp"

namespace mapping_panel
{

MappingPanel::MappingPanel(QWidget *parent) : rviz_common::Panel(parent)
{  
  //@---------- operation button
  QGroupBox *operation_groupBox = new QGroupBox(tr("Mapping Panel"));
  pause_btn_ = new QPushButton(this);
  pause_btn_->setText("Pause");
  connect(pause_btn_, SIGNAL(clicked()), this, SLOT(pause()));

  resume_btn_ = new QPushButton(this);
  resume_btn_->setText("Resume");
  connect(resume_btn_, SIGNAL(clicked()), this, SLOT(resume()));

  //addWidget(*Widget, row, column, rowspan, colspan)
  QGridLayout *grid_layout = new QGridLayout;
  grid_layout->addWidget(pause_btn_, 0, 0, 1, 1);
  grid_layout->addWidget(resume_btn_, 0, 1, 1, 1);
  operation_groupBox->setLayout(grid_layout);
  
  //ICP Score slider
  QGroupBox *icp_scoreBox = new QGroupBox(tr("ICP Score"));
  icp_score_slider_ = new QSlider(Qt::Horizontal,0);
  icp_score_slider_->setFocusPolicy(Qt::StrongFocus);
  icp_score_slider_->setMinimum(1.0);
  icp_score_slider_->setMaximum(100.0);
  icp_score_slider_->setValue(10.0);
  icp_score_slider_->setTickPosition(QSlider::TicksBelow);
  icp_score_slider_->setTickInterval(1.0);
  icp_score_slider_->setSingleStep(1.0);  
  connect(icp_score_slider_, &QSlider::valueChanged, this, &MappingPanel::setICPScoreValue);
  icp_score_value_ = new QLabel(this);
  icp_score_value_->setText(std::to_string(icp_score_slider_->value()*0.1).c_str());
  QVBoxLayout *icp_score_vbox = new QVBoxLayout;
  icp_score_vbox->addWidget(icp_score_slider_);
  icp_score_vbox->addWidget(icp_score_value_);
  icp_score_vbox->addStretch(1);
  icp_scoreBox->setLayout(icp_score_vbox);

  //History Keyframe Search Radius slider
  QGroupBox *history_keyframe_search_radiusBox = new QGroupBox(tr("History Keyframe Search Radius"));
  history_keyframe_search_radius_slider_ = new QSlider(Qt::Horizontal,0);
  history_keyframe_search_radius_slider_->setFocusPolicy(Qt::StrongFocus);
  history_keyframe_search_radius_slider_->setMinimum(1.0);
  history_keyframe_search_radius_slider_->setMaximum(100.0);
  history_keyframe_search_radius_slider_->setValue(10.0);
  history_keyframe_search_radius_slider_->setTickPosition(QSlider::TicksBelow);
  history_keyframe_search_radius_slider_->setTickInterval(1.0);
  history_keyframe_search_radius_slider_->setSingleStep(1.0);  
  connect(history_keyframe_search_radius_slider_, &QSlider::valueChanged, this, &MappingPanel::setHistoryKeyframeSearchRadiusValue);
  history_keyframe_search_radius_value_ = new QLabel(this);
  history_keyframe_search_radius_value_->setText(std::to_string(history_keyframe_search_radius_slider_->value()).c_str());
  QVBoxLayout *history_keyframe_search_radius_vbox = new QVBoxLayout;
  history_keyframe_search_radius_vbox->addWidget(history_keyframe_search_radius_slider_);
  history_keyframe_search_radius_vbox->addWidget(history_keyframe_search_radius_value_);
  history_keyframe_search_radius_vbox->addStretch(1);
  history_keyframe_search_radiusBox->setLayout(history_keyframe_search_radius_vbox);

  //Skip Frame slider
  QGroupBox *skip_frameBox = new QGroupBox(tr("Skip Frame"));
  skip_frame_slider_ = new QSlider(Qt::Horizontal,0);
  skip_frame_slider_->setFocusPolicy(Qt::StrongFocus);
  skip_frame_slider_->setMinimum(1.0);
  skip_frame_slider_->setMaximum(100.0);
  skip_frame_slider_->setValue(10.0);
  skip_frame_slider_->setTickPosition(QSlider::TicksBelow);
  skip_frame_slider_->setTickInterval(1.0);
  skip_frame_slider_->setSingleStep(1.0);  
  connect(skip_frame_slider_, &QSlider::valueChanged, this, &MappingPanel::setSkipFrameValue);
  skip_frame_value_ = new QLabel(this);
  skip_frame_value_->setText(std::to_string(skip_frame_slider_->value()).c_str());
  QVBoxLayout *skip_frame_vbox = new QVBoxLayout;
  skip_frame_vbox->addWidget(skip_frame_slider_);
  skip_frame_vbox->addWidget(skip_frame_value_);
  skip_frame_vbox->addStretch(1);
  skip_frameBox->setLayout(skip_frame_vbox);

  //@ add everything to layout
  auto layout = new QVBoxLayout();
  layout->setContentsMargins(0, 0, 0, 0);
  layout->addWidget(operation_groupBox);
  layout->addWidget(icp_scoreBox);
  layout->addWidget(history_keyframe_search_radiusBox);
  layout->addWidget(skip_frameBox);
  setLayout(layout);
  
}

void MappingPanel::onInitialize()
{
  //@ ros stuff can only be placed here instead of in constructor
  auto raw_node = getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node();
  pause_resume_pub_ = raw_node->create_publisher<std_msgs::msg::Bool>("lego_loam_bag_pause", 1);
  icp_score_pub_ = raw_node->create_publisher<std_msgs::msg::Float32>("lego_loam_bag_icp_score", 1);
  history_keyframe_search_radius_pub_ = raw_node->create_publisher<std_msgs::msg::Float32>("lego_loam_bag_history_keyframe_search_radius", 1);
  skip_frame_pub_ = raw_node->create_publisher<std_msgs::msg::Int32>("lego_loam_bag_skip_frame", 1);
  clock_ = raw_node->get_clock();

}

void MappingPanel::pause(){
  std_msgs::msg::Bool tmp_bool;
  tmp_bool.data = true;
  pause_resume_pub_->publish(tmp_bool);
}

void MappingPanel::resume(){
  std_msgs::msg::Bool tmp_bool;
  tmp_bool.data = false;
  pause_resume_pub_->publish(tmp_bool);
}

void MappingPanel::setICPScoreValue(){
  icp_score_value_->setText(std::to_string(icp_score_slider_->value()*0.1).c_str());
  std_msgs::msg::Float32 tmp_float;
  tmp_float.data = icp_score_slider_->value()*0.1;
  icp_score_pub_->publish(tmp_float);
}

void MappingPanel::setHistoryKeyframeSearchRadiusValue(){
  history_keyframe_search_radius_value_->setText(std::to_string(history_keyframe_search_radius_slider_->value()).c_str());
  std_msgs::msg::Float32 tmp_float;
  tmp_float.data = history_keyframe_search_radius_slider_->value();
  history_keyframe_search_radius_pub_->publish(tmp_float);
}

void MappingPanel::setSkipFrameValue(){
  skip_frame_value_->setText(std::to_string(skip_frame_slider_->value()).c_str());
  std_msgs::msg::Int32 tmp_int;
  tmp_int.data = skip_frame_slider_->value();
  skip_frame_pub_->publish(tmp_int);
}

}  // namespace mapping_panel


#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(mapping_panel::MappingPanel, rviz_common::Panel)
