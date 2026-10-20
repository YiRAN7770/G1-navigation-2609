#pragma once

// Shared by planner and follower. Distances refer to the original map goal,
// never the FAR projected waypoint.
namespace final_goal {
constexpr double reverseDistance = 0.50;
constexpr double alignDistance = 0.20;
constexpr double correctionExitDistance = 0.30;
constexpr double alignReleaseYaw = 0.17453292519943295;  // 10 degrees
constexpr double maxTranslationSpeed = 0.08;
constexpr double maxLateralSpeed = 0.05;
constexpr double maxAlignmentRate = 0.15;
constexpr double pathTimeout = 0.5;
constexpr double footprintMargin = 0.03;
}
