#include <prometheus/ut/unit_test.hpp>

using namespace prometheus;

auto main() noexcept -> int
{
	// ut::get_config().report_level = ut::config_type::ReportLevel::ALL;
	ut::get_config().break_point_level = ut::config_type::BreakPointLevel::FAILURE;
}
