#include "EnergyResidual.h"

#include <mc_control/GlobalPluginMacros.h>

namespace mc_plugin
{

EnergyResidual::~EnergyResidual() = default;

void EnergyResidual::init(mc_control::MCGlobalController & controller, const mc_rtc::Configuration & config)
{
  mc_rtc::log::info("EnergyResidual::init called with configuration:\n{}", config.dump(true, true));
}

void EnergyResidual::reset(mc_control::MCGlobalController & controller)
{
  mc_rtc::log::info("EnergyResidual::reset called");
}

void EnergyResidual::before(mc_control::MCGlobalController &)
{
  mc_rtc::log::info("EnergyResidual::before");
}

void EnergyResidual::after(mc_control::MCGlobalController & controller)
{
  mc_rtc::log::info("EnergyResidual::after");
}

mc_control::GlobalPlugin::GlobalPluginConfiguration EnergyResidual::configuration()
{
  mc_control::GlobalPlugin::GlobalPluginConfiguration out;
  out.should_run_before = true;
  out.should_run_after = true;
  out.should_always_run = true;
  return out;
}

} // namespace mc_plugin

EXPORT_MC_RTC_PLUGIN("EnergyResidual", mc_plugin::EnergyResidual)
