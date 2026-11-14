// Copyright 2026 Angelos Zacharia

#include "gbplanner_control_panel/gbplanner_control_panel.hpp"

#include <QPushButton>
#include <QTimer>
#include <QVBoxLayout>

#include <array>
#include <exception>
#include <memory>
#include <string>
#include <type_traits>

#include "pluginlib/class_list_macros.hpp"
#include "rviz_common/display_context.hpp"

namespace gbplanner_control_panel
{
namespace
{

constexpr std::array<const char *, 4> kTriggerServiceNames = {
  "/planner_control_interface/std_srvs/automatic_planning",
  "/planner_control_interface/std_srvs/single_planning",
  "/planner_control_interface/std_srvs/stop",
  "/planner_control_interface/std_srvs/homing_trigger"
};

constexpr std::array<const char *, 6> kButtonLabels = {
  "Start Planner", "Start Single Planner", "Stop Planner", "Go Home",
  "Initialization", "Operation Mode (EXP)"
};

}  // namespace

GbplannerControlPanel::GbplannerControlPanel(QWidget * parent)
: rviz_common::Panel(parent),
  service_timer_(new QTimer(this))
{
  auto * layout = new QVBoxLayout(this);
  layout->setContentsMargins(6, 6, 6, 6);
  layout->setSpacing(4);
  layout->setSizeConstraint(QLayout::SetMinimumSize);
  for (std::size_t index = 0; index < kActionCount; ++index) {
    buttons_[index] = new QPushButton(kButtonLabels[index], this);
    buttons_[index]->setEnabled(false);
  }
  for (std::size_t index = 0; index < kActionCount; ++index) {
    layout->addWidget(buttons_[index]);
  }
  for (std::size_t index = 0; index < kTriggerCount; ++index) {
    connect(buttons_[index], &QPushButton::clicked, this,
      [this, index]() {sendTrigger(index);});
  }
  connect(buttons_[4], &QPushButton::clicked, this,
    [this]() {sendInitialization();});
  connect(buttons_[5], &QPushButton::clicked, this,
    [this]() {sendOperationMode();});
  buttons_[4]->setToolTip("Trigger PCI's configured arm and takeoff motion.");

  connect(service_timer_, &QTimer::timeout, this,
    [this]() {refreshServices();});
  service_timer_->setInterval(1000);

  connect(this, &GbplannerControlPanel::requestFinished, this,
    [this](int action, qulonglong generation, bool success, const QString & message) {
      const auto index = static_cast<std::size_t>(action);
      if (index >= kActionCount || !pending_[index] || generations_[index] != generation) {
        return;
      }
      pending_[index] = false;
      if (success && index == 3) {
        waypoint_navigation_mode_ = false;
        buttons_[5]->setText("Operation Mode (EXP)");
      } else if (success && index == 5) {
        waypoint_navigation_mode_ = !waypoint_navigation_mode_;
        buttons_[5]->setText(
          waypoint_navigation_mode_ ? "Operation Mode (WP)" : "Operation Mode (EXP)");
      }
      if (!success) {
        const auto failure_message = message.toStdString();
        RCLCPP_WARN(node_->get_logger(), "%s request failed: %s",
          kButtonLabels[index], failure_message.c_str());
      }
      refreshServices();
    }, Qt::QueuedConnection);
}

void GbplannerControlPanel::onInitialize()
{
  const auto abstraction = getDisplayContext()->getRosNodeAbstraction().lock();
  if (!abstraction) {
    return;
  }

  node_ = abstraction->get_raw_node();
  for (std::size_t index = 0; index < kTriggerCount; ++index) {
    trigger_clients_[index] = node_->create_client<Trigger>(kTriggerServiceNames[index]);
  }
  initialization_client_ = node_->create_client<PciInitialization>(
    "/pci_initialization_trigger");
  operation_mode_client_ = node_->create_client<SetBool>(
    "/gbplanner/switch_operation_mode");

  refreshServices();
  service_timer_->start();
}

void GbplannerControlPanel::sendTrigger(std::size_t action)
{
  sendRequest(action, trigger_clients_[action], std::make_shared<Trigger::Request>());
}

void GbplannerControlPanel::sendInitialization()
{
  sendRequest(4, initialization_client_,
    std::make_shared<PciInitialization::Request>());
}

void GbplannerControlPanel::sendOperationMode()
{
  auto request = std::make_shared<SetBool::Request>();
  request->data = !waypoint_navigation_mode_;
  sendRequest(5, operation_mode_client_, request);
}

template<typename Service>
void GbplannerControlPanel::sendRequest(
  std::size_t action,
  const std::shared_ptr<rclcpp::Client<Service>> & client,
  const std::shared_ptr<typename Service::Request> & request)
{
  if (!client || !client->service_is_ready() || pending_[action]) {
    return;
  }

  const auto generation = ++generations_[action];
  pending_[action] = true;
  refreshServices();

  QPointer<GbplannerControlPanel> guarded_panel(this);
  try {
    const auto pending_request = client->async_send_request(
      request,
      [guarded_panel, action, generation](typename rclcpp::Client<Service>::SharedFuture future) {
        if (!guarded_panel) {
          return;
        }
        try {
          const auto response = future.get();
          QString message;
          if constexpr (
            std::is_same_v<Service, Trigger> || std::is_same_v<Service, SetBool>)
          {
            message = QString::fromStdString(response->message);
          }
          Q_EMIT guarded_panel->requestFinished(
            static_cast<int>(action), generation, response->success, message);
        } catch (const std::exception & error) {
          Q_EMIT guarded_panel->requestFinished(
            static_cast<int>(action), generation, false, QString::fromUtf8(error.what()));
        }
      });
    const auto request_id = pending_request.request_id;
    QTimer::singleShot(30000, this, [this, action, generation, client, request_id]() {
      if (!pending_[action] || generations_[action] != generation) {
        return;
      }
      client->remove_pending_request(request_id);
      pending_[action] = false;
      RCLCPP_WARN(node_->get_logger(), "%s request timed out", kButtonLabels[action]);
      refreshServices();
    });
  } catch (const std::exception & error) {
    pending_[action] = false;
    RCLCPP_WARN(node_->get_logger(), "%s request failed: %s",
      kButtonLabels[action], error.what());
    refreshServices();
    return;
  }

}

void GbplannerControlPanel::refreshServices()
{
  for (std::size_t index = 0; index < kTriggerCount; ++index) {
    const bool available =
      trigger_clients_[index] && trigger_clients_[index]->service_is_ready();
    buttons_[index]->setEnabled(available && !pending_[index]);
  }
  const std::array<bool, 2> custom_available = {
    initialization_client_ && initialization_client_->service_is_ready(),
    operation_mode_client_ && operation_mode_client_->service_is_ready()
  };
  for (std::size_t index = 0; index < custom_available.size(); ++index) {
    const auto action = kTriggerCount + index;
    buttons_[action]->setEnabled(custom_available[index] && !pending_[action]);
  }
}

}  // namespace gbplanner_control_panel

PLUGINLIB_EXPORT_CLASS(
  gbplanner_control_panel::GbplannerControlPanel, rviz_common::Panel)
