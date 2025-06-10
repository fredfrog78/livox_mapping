#include "gtest/gtest.h"
#include "rclcpp/rclcpp.hpp"
#include "adaptive_parameter_manager.h" // Assuming src is in include path
#include "adaptive_parameter_manager_types.h" // Assuming src is in include path

// To allow tests to access private/protected members for direct manipulation and observation:
// Option 1: Make test a friend class (modify adaptive_parameter_manager.h)
// Option 2: Use a derived class for testing (if methods are virtual)
// Option 3: For this subtask, we will call public methods and infer state,
//           or assume some helper methods could be temporarily made public / testable if needed.
//           Ideally, the class would be designed for testability.

using namespace loam_adaptive_parameter_manager;

class TestableAdaptiveParameterManager : public AdaptiveParameterManager {
public:
    TestableAdaptiveParameterManager() : AdaptiveParameterManager() {}

    // Expose internal state for testing determination logic
    void setStabilizedHealth(ScanRegistrationHealth sr_h, LaserMappingHealth lm_h) {
        stabilized_sr_health_ = sr_h;
        stabilized_lm_health_ = lm_h;
    }

    void setResourceMetrics(float cpu, float mem, float latency, bool fresh = true) {
        latest_cpu_load_ = cpu;
        latest_memory_usage_ = mem;
        latest_pipeline_latency_sec_ = latency;

        rclcpp::Time now_time = this->now(); // Use node's clock
        if (!fresh) {
             // Make them stale by subtracting more than stale threshold
             // Ensure metric_stale_threshold_sec_ is initialized before calling this when fresh=false
             double stale_offset = this->metric_stale_threshold_sec_ + 1.0;
             last_cpu_load_timestamp_ = now_time - rclcpp::Duration::from_seconds(stale_offset);
             last_memory_usage_timestamp_ = now_time - rclcpp::Duration::from_seconds(stale_offset);
             last_pipeline_latency_timestamp_ = now_time - rclcpp::Duration::from_seconds(stale_offset);
        } else {
             last_cpu_load_timestamp_ = now_time;
             last_memory_usage_timestamp_ = now_time;
             last_pipeline_latency_timestamp_ = now_time;
        }
    }

    SystemHealth getSystemHealth() { determineSystemHealth(); return current_system_health_; }
    int getICPIterations() { return current_icp_iterations_; }
    double getCornerFilter() { return current_filter_parameter_corner_; }
    double getSurfFilter() { return current_filter_parameter_surf_; }
    bool isCooldownActive() { return overload_cooldown_active_; }
    int getConsecutiveHealthyCycles() { return consecutive_healthy_cycles_; }
    int getConsecutiveICPIssueWarnings() { return consecutive_icp_issue_warnings_; }


    // Expose parameter loading for testing defaults
    void publicInitializeParameters() { initializeParameters(); }

    // Expose internal thresholds for verification against defaults
    double getCpuHighThreshold() { return cpu_load_threshold_high_; }
    // Add other getters for private members if needed for tests
    int getMinICPIterations() { return min_icp_iterations_; }
    int getMaxICPIterations() { return max_icp_iterations_; }
    int getDefaultICPIterations() { return default_icp_iterations_; }
    int getICPIterationAdjustmentStep() { return icp_iteration_adjustment_step_; }

    // Making const values accessible for tests - No longer needed here, access directly from AdaptiveParameterManager public static members
    // static const int TEST_ICP_ISSUE_THRESHOLD_FOR_OVERLOAD_ = 3;
    // static const int TEST_HEALTHY_CYCLES_TO_RESET_COOLDOWN_ = 5;
    // static const int TEST_PROBING_AFTER_N_HEALTHY_CYCLES_ = 2;

};

class AdaptiveParameterManagerTest : public ::testing::Test {
protected:
    std::shared_ptr<TestableAdaptiveParameterManager> apm_;
    // rclcpp::Node::SharedPtr test_node_; // APM is a Node, so it can provide its own clock, no need for separate test_node_

    void SetUp() override {
        if (!rclcpp::ok()) { // Changed from is_initialized
            rclcpp::init(0, nullptr);
        }

        apm_ = std::make_shared<TestableAdaptiveParameterManager>();
        // Parameters are declared in constructor, then values loaded in initializeParameters.
        // Call initialize manually to load default ROS params (using defaults coded in declare_parameter)
        apm_->publicInitializeParameters(); // Ensure defaults are loaded
    }

    void TearDown() override {
        if (rclcpp::ok()) { // Changed from is_initialized
            rclcpp::shutdown();
        }
    }
};

TEST_F(AdaptiveParameterManagerTest, InitializationLoadsDefaultParams) {
    ASSERT_DOUBLE_EQ(apm_->getCpuHighThreshold(), 0.85);
    ASSERT_EQ(apm_->getICPIterations(), apm_->getDefaultICPIterations());
}

TEST_F(AdaptiveParameterManagerTest, DetermineHealth_AllHealthy) {
    apm_->setStabilizedHealth(ScanRegistrationHealth::HEALTHY, LaserMappingHealth::HEALTHY);
    apm_->setResourceMetrics(0.1f, 0.1f, 0.1f);
    EXPECT_EQ(apm_->getSystemHealth(), SystemHealth::HEALTHY);
}

TEST_F(AdaptiveParameterManagerTest, DetermineHealth_ICPUnstable) {
    apm_->setStabilizedHealth(ScanRegistrationHealth::HEALTHY, LaserMappingHealth::ICP_DEGENERATE);
    apm_->setResourceMetrics(0.1f, 0.1f, 0.1f);
    EXPECT_EQ(apm_->getSystemHealth(), SystemHealth::LASER_MAPPING_ICP_UNSTABLE);
}

TEST_F(AdaptiveParameterManagerTest, DetermineHealth_StaleMetricsStillAllowsICPUnstable) {
    apm_->setStabilizedHealth(ScanRegistrationHealth::HEALTHY, LaserMappingHealth::ICP_DEGENERATE);
    apm_->setResourceMetrics(0.1f, 0.1f, 0.1f, false);
    EXPECT_EQ(apm_->getSystemHealth(), SystemHealth::LASER_MAPPING_ICP_UNSTABLE);
}

TEST_F(AdaptiveParameterManagerTest, DetermineHealth_CriticalCPULoad) {
    apm_->setStabilizedHealth(ScanRegistrationHealth::HEALTHY, LaserMappingHealth::HEALTHY);
    apm_->setResourceMetrics(apm_->cpu_load_threshold_critical_ + 0.01f, 0.1f, 0.1f); // Critical CPU
    EXPECT_EQ(apm_->getSystemHealth(), SystemHealth::HIGH_CPU_LOAD);
}

TEST_F(AdaptiveParameterManagerTest, DetermineHealth_HighLatency) {
    apm_->setStabilizedHealth(ScanRegistrationHealth::HEALTHY, LaserMappingHealth::HEALTHY);
    apm_->setResourceMetrics(0.1f, 0.1f, apm_->pipeline_latency_threshold_high_sec_ + 0.1f);
    EXPECT_EQ(apm_->getSystemHealth(), SystemHealth::PIPELINE_FALLING_BEHIND);
}

TEST_F(AdaptiveParameterManagerTest, DetermineHealth_StaleMetricsHealthyIfSRLMHealthy) {
    apm_->setStabilizedHealth(ScanRegistrationHealth::HEALTHY, LaserMappingHealth::HEALTHY);
    apm_->setResourceMetrics(0.1f, 0.1f, 0.1f, false); // All metrics stale
    EXPECT_EQ(apm_->getSystemHealth(), SystemHealth::HEALTHY);
}

TEST_F(AdaptiveParameterManagerTest, Process_Healthy_ProbesICPIterationsUp) {
    apm_->setStabilizedHealth(ScanRegistrationHealth::HEALTHY, LaserMappingHealth::HEALTHY);
    apm_->setResourceMetrics(0.1f, 0.1f, 0.1f);
    int initial_icp = apm_->getICPIterations();
    for (int i = 0; i < AdaptiveParameterManager::PROBING_AFTER_N_HEALTHY_CYCLES_; ++i) {
        apm_->processHealthAndAdjustParameters();
        ASSERT_EQ(apm_->getConsecutiveHealthyCycles(), i + 1);
    }
    apm_->processHealthAndAdjustParameters();
    EXPECT_GT(apm_->getICPIterations(), initial_icp);
    EXPECT_EQ(apm_->getICPIterations(), initial_icp + apm_->getICPIterationAdjustmentStep());
}

TEST_F(AdaptiveParameterManagerTest, Process_Healthy_ProbesFiltersDownAfterMaxICP) {
    apm_->setStabilizedHealth(ScanRegistrationHealth::HEALTHY, LaserMappingHealth::HEALTHY);
    apm_->setResourceMetrics(0.1f, 0.1f, 0.1f);
    apm_->current_icp_iterations_ = apm_->getMaxICPIterations();
    double initial_corner = apm_->getCornerFilter();
    double initial_surf = apm_->getSurfFilter();

    for (int i = 0; i < AdaptiveParameterManager::PROBING_AFTER_N_HEALTHY_CYCLES_; ++i) {
        apm_->processHealthAndAdjustParameters();
    }
    apm_->processHealthAndAdjustParameters();

    EXPECT_EQ(apm_->getICPIterations(), apm_->getMaxICPIterations());
    EXPECT_LT(apm_->getCornerFilter(), initial_corner);
    EXPECT_LT(apm_->getSurfFilter(), initial_surf);
}

TEST_F(AdaptiveParameterManagerTest, Process_HighCPU_ReducesICPIterations) {
    apm_->setStabilizedHealth(ScanRegistrationHealth::HEALTHY, LaserMappingHealth::HEALTHY);
    apm_->setResourceMetrics(apm_->cpu_load_threshold_high_ + 0.05f, 0.1f, 0.1f);
    int initial_icp = apm_->getICPIterations();
    apm_->processHealthAndAdjustParameters();
    EXPECT_EQ(apm_->getSystemHealth(), SystemHealth::HIGH_CPU_LOAD);
    EXPECT_LT(apm_->getICPIterations(), initial_icp);
    EXPECT_EQ(apm_->getICPIterations(), initial_icp - apm_->getICPIterationAdjustmentStep());
}

TEST_F(AdaptiveParameterManagerTest, Process_HighCPU_ThenFiltersIfICPatMin) {
    apm_->setStabilizedHealth(ScanRegistrationHealth::HEALTHY, LaserMappingHealth::HEALTHY);
    apm_->setResourceMetrics(apm_->cpu_load_threshold_high_ + 0.05f, 0.1f, 0.1f);
    apm_->current_icp_iterations_ = apm_->getMinICPIterations();

    double initial_corner = apm_->getCornerFilter();
    double initial_surf = apm_->getSurfFilter();

    apm_->processHealthAndAdjustParameters();
    EXPECT_EQ(apm_->getSystemHealth(), SystemHealth::HIGH_CPU_LOAD);
    EXPECT_EQ(apm_->getICPIterations(), apm_->getMinICPIterations());
    EXPECT_GT(apm_->getCornerFilter(), initial_corner);
    EXPECT_GT(apm_->getSurfFilter(), initial_surf);
}

TEST_F(AdaptiveParameterManagerTest, Process_ICPUnstable_TriggersCooldown) {
    apm_->setStabilizedHealth(ScanRegistrationHealth::HEALTHY, LaserMappingHealth::ICP_DEGENERATE);
    apm_->setResourceMetrics(0.1f, 0.1f, 0.1f);

    for (int i = 0; i < AdaptiveParameterManager::ICP_ISSUE_THRESHOLD_FOR_OVERLOAD_ -1; ++i) {
        apm_->processHealthAndAdjustParameters();
        ASSERT_FALSE(apm_->isCooldownActive());
        ASSERT_EQ(apm_->getConsecutiveICPIssueWarnings(), i+1);
    }
    apm_->processHealthAndAdjustParameters();
    ASSERT_TRUE(apm_->isCooldownActive());
    EXPECT_LT(apm_->getICPIterations(), apm_->getDefaultICPIterations());
}

TEST_F(AdaptiveParameterManagerTest, Process_CooldownResetsAfterHealthyCycles) {
    apm_->setStabilizedHealth(ScanRegistrationHealth::HEALTHY, LaserMappingHealth::ICP_DEGENERATE);
    apm_->setResourceMetrics(0.1f, 0.1f, 0.1f);
    for (int i = 0; i < AdaptiveParameterManager::ICP_ISSUE_THRESHOLD_FOR_OVERLOAD_; ++i) {
        apm_->processHealthAndAdjustParameters();
    }
    ASSERT_TRUE(apm_->isCooldownActive());

    apm_->setStabilizedHealth(ScanRegistrationHealth::HEALTHY, LaserMappingHealth::HEALTHY);
    for (int i = 0; i < AdaptiveParameterManager::HEALTHY_CYCLES_TO_RESET_COOLDOWN_; ++i) {
        apm_->processHealthAndAdjustParameters();
        if (i < AdaptiveParameterManager::HEALTHY_CYCLES_TO_RESET_COOLDOWN_ - 1) { // Check before the last cycle
           ASSERT_TRUE(apm_->isCooldownActive());
        }
    }
    EXPECT_FALSE(apm_->isCooldownActive());
    EXPECT_EQ(apm_->getICPIterations(), apm_->getDefaultICPIterations());
}

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    // Initialize rclcpp once for all tests, if not already handled by SetUp/TearDown
    // rclcpp::init(argc, argv); // This might be problematic if called multiple times
    // int ret = RUN_ALL_TESTS();
    // rclcpp::shutdown();
    // return ret;
    return RUN_ALL_TESTS(); // Simpler, relying on SetUp/TearDown for init/shutdown per test fixture
}
