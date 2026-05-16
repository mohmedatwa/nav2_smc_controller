#include <gtest/gtest.h>
#include <cmath>
#include "nav2_smc_controller/smc_controller.hpp"

using nav2_smc_controller::SMCController;

// ═══════════════════════════════════════════════════════════════════════════
// sat() — boundary-layer saturation
// ═══════════════════════════════════════════════════════════════════════════

TEST(SMCUtils, sat_inside_boundary_layer)
{
  // Linear region: s / phi should pass through unchanged
  EXPECT_DOUBLE_EQ(SMCController::sat(0.0,  0.1),  0.0);
  EXPECT_DOUBLE_EQ(SMCController::sat(0.05, 0.1),  0.5);
  EXPECT_DOUBLE_EQ(SMCController::sat(-0.05, 0.1), -0.5);
}

TEST(SMCUtils, sat_outside_boundary_layer)
{
  // Saturated region: output must be clamped to ±1
  EXPECT_DOUBLE_EQ(SMCController::sat( 0.2, 0.1),  1.0);
  EXPECT_DOUBLE_EQ(SMCController::sat(-0.2, 0.1), -1.0);
  EXPECT_DOUBLE_EQ(SMCController::sat( 1e6, 0.1),  1.0);
  EXPECT_DOUBLE_EQ(SMCController::sat(-1e6, 0.1), -1.0);
}

TEST(SMCUtils, sat_exactly_at_boundary)
{
  // s == ±phi → exactly ±1 (boundary of linear region)
  EXPECT_DOUBLE_EQ(SMCController::sat( 0.1, 0.1),  1.0);
  EXPECT_DOUBLE_EQ(SMCController::sat(-0.1, 0.1), -1.0);
}

TEST(SMCUtils, sat_phi_near_zero_positive_s)
{
  // phi < 1e-9 → pure sign function
  EXPECT_DOUBLE_EQ(SMCController::sat( 1.0, 1e-10),  1.0);
  EXPECT_DOUBLE_EQ(SMCController::sat( 1e-6, 1e-10), 1.0);
}

TEST(SMCUtils, sat_phi_near_zero_negative_s)
{
  EXPECT_DOUBLE_EQ(SMCController::sat(-1.0,  1e-10), -1.0);
  EXPECT_DOUBLE_EQ(SMCController::sat(-1e-6, 1e-10), -1.0);
}

TEST(SMCUtils, sat_phi_zero_exactly)
{
  // phi == 0.0 triggers the phi < 1e-9 branch.
  // s = 0.0 is >= 0, so returns +1 (discontinuity at origin — documented behavior).
  EXPECT_DOUBLE_EQ(SMCController::sat(0.0, 0.0), 1.0);
}

TEST(SMCUtils, sat_negative_phi)
{
  // Negative phi must behave identically to |phi|.
  // Without std::abs in the implementation this returns -1 for positive s — a bug.
  // This test enforces the correct contract.
  EXPECT_DOUBLE_EQ(SMCController::sat( 0.05, -0.1),  0.5);
  EXPECT_DOUBLE_EQ(SMCController::sat( 0.2,  -0.1),  1.0);
  EXPECT_DOUBLE_EQ(SMCController::sat(-0.2,  -0.1), -1.0);
}

// ═══════════════════════════════════════════════════════════════════════════
// wrapAngle() — wraps to (-π, π]
// ═══════════════════════════════════════════════════════════════════════════

TEST(SMCUtils, wrapAngle_already_in_range)
{
  EXPECT_NEAR(SMCController::wrapAngle(0.0),        0.0,  1e-9);
  EXPECT_NEAR(SMCController::wrapAngle(1.5),        1.5,  1e-9);
  EXPECT_NEAR(SMCController::wrapAngle(-1.5),      -1.5,  1e-9);
}

TEST(SMCUtils, wrapAngle_just_above_pi)
{
  // 3.2 > π  →  3.2 − 2π ≈ −3.083  (still in range, no second wrap)
  EXPECT_NEAR(SMCController::wrapAngle(3.2), 3.2 - 2.0 * M_PI, 1e-9);
}

TEST(SMCUtils, wrapAngle_just_below_neg_pi)
{
  // −3.2 < −π  →  −3.2 + 2π ≈ 3.083
  EXPECT_NEAR(SMCController::wrapAngle(-3.2), -3.2 + 2.0 * M_PI, 1e-9);
}

TEST(SMCUtils, wrapAngle_boundary_plus_pi)
{
  // Exactly +π: condition is (a > M_PI), so +π is NOT wrapped — stays as +π.
  EXPECT_NEAR(SMCController::wrapAngle(M_PI),  M_PI,  1e-9);
}

TEST(SMCUtils, wrapAngle_boundary_minus_pi)
{
  // Exactly −π: condition is (a < −M_PI), so −π is NOT wrapped — stays as −π.
  EXPECT_NEAR(SMCController::wrapAngle(-M_PI), -M_PI, 1e-9);
}

TEST(SMCUtils, wrapAngle_multiple_wraps_positive)
{
  // 5π wraps twice → π (5π − 2π = 3π − 2π = π)
  EXPECT_NEAR(SMCController::wrapAngle(5.0 * M_PI),  M_PI,  1e-9);
  EXPECT_NEAR(SMCController::wrapAngle(2.0 * M_PI),  0.0,   1e-9);
  EXPECT_NEAR(SMCController::wrapAngle(4.0 * M_PI),  0.0,   1e-9);
}

TEST(SMCUtils, wrapAngle_multiple_wraps_negative)
{
  EXPECT_NEAR(SMCController::wrapAngle(-5.0 * M_PI), -M_PI, 1e-9);
  EXPECT_NEAR(SMCController::wrapAngle(-2.0 * M_PI),  0.0,  1e-9);
  EXPECT_NEAR(SMCController::wrapAngle(-4.0 * M_PI),  0.0,  1e-9);
}
