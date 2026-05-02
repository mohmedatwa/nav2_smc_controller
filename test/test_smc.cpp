#include <gtest/gtest.h>
#include "nav2_smc_controller/smc_controller.hpp"

TEST(SMCUtils, sat)
{
    using nav2_smc_controller::SMCController;
    EXPECT_DOUBLE_EQ(SMCController::sat(0.0, 0.1), 0.0);
    EXPECT_DOUBLE_EQ(SMCController::sat(0.2, 0.1), 1.0);
    EXPECT_DOUBLE_EQ(SMCController::sat(-0.2, 0.1), -1.0);
    EXPECT_DOUBLE_EQ(SMCController::sat(0.05, 0.1), 0.5);
    // phi near zero
    EXPECT_DOUBLE_EQ(SMCController::sat(1.0, 1e-10), 1.0);
    EXPECT_DOUBLE_EQ(SMCController::sat(-1.0, 1e-10), -1.0);
}

TEST(SMCUtils, wrapAngle)
{
    using nav2_smc_controller::SMCController;
    EXPECT_NEAR(SMCController::wrapAngle(3.2), 3.2 - 2 * M_PI, 1e-6);
    EXPECT_NEAR(SMCController::wrapAngle(-3.2), -3.2 + 2 * M_PI, 1e-6);
    EXPECT_NEAR(SMCController::wrapAngle(0.0), 0.0, 1e-6);
}
