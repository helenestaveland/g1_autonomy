// Copyright 2026 Angelos Zacharia

#ifndef GBPLANNER_CONTROL_PANEL__GBPLANNER_CONTROL_PANEL_HPP_
#define GBPLANNER_CONTROL_PANEL__GBPLANNER_CONTROL_PANEL_HPP_

#include <QPointer>
#include <QString>

#include <array>
#include <cstddef>
#include <cstdint>
#include <memory>

#include "planner_msgs/srv/pci_initialization.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rviz_common/panel.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "std_srvs/srv/trigger.hpp"

class QPushButton;
class QTimer;

namespace gbplanner_control_panel
{

class GbplannerControlPanel : public rviz_common::Panel
{
  Q_OBJECT

public:
  explicit GbplannerControlPanel(QWidget * parent = nullptr);
  void onInitialize() override;

Q_SIGNALS:
  void requestFinished(int action, qulonglong generation, bool success, const QString & message);

private:
  using Trigger = std_srvs::srv::Trigger;
  using SetBool = std_srvs::srv::SetBool;
  using PciInitialization = planner_msgs::srv::PciInitialization;
  static constexpr std::size_t kTriggerCount = 4;
  static constexpr std::size_t kActionCount = 6;

  void sendTrigger(std::size_t action);
  void sendInitialization();
  void sendOperationMode();
  template<typename Service>
  void sendRequest(
    std::size_t action,
    const std::shared_ptr<rclcpp::Client<Service>> & client,
    const std::shared_ptr<typename Service::Request> & request);
  void refreshServices();

  QTimer * service_timer_;
  std::array<QPushButton *, kActionCount> buttons_;
  std::array<rclcpp::Client<Trigger>::SharedPtr, kTriggerCount> trigger_clients_;
  rclcpp::Client<PciInitialization>::SharedPtr initialization_client_;
  rclcpp::Client<SetBool>::SharedPtr operation_mode_client_;
  std::array<qulonglong, kActionCount> generations_{};
  std::array<bool, kActionCount> pending_{};
  bool waypoint_navigation_mode_ = false;
  rclcpp::Node::SharedPtr node_;
};

}  // namespace gbplanner_control_panel

#endif  // GBPLANNER_CONTROL_PANEL__GBPLANNER_CONTROL_PANEL_HPP_
