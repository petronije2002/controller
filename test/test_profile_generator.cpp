#include <unity.h>
#include "ProfileGenerator.h"

// Create a ProfileGenerator instance
ProfileGenerator* profileGen;

void setUp() {
    // This is called before each test
    profileGen = new ProfileGenerator(4.0f, 2.0f, 50);  // maxVelocity, maxAcceleration, numSegments
}

void tearDown() {
    // This is called after each test
    delete profileGen;
}

void test_generateScurveProfile() {
    // Test generating the S-curve profile
    float totalDistance = 2.0f;
    float commandedVelocity = 3.0f;
    profileGen->generateScurveProfile(totalDistance, commandedVelocity);

    const std::vector<float>& positionProfile = profileGen->getPositionProfile();
    const std::vector<float>& velocityProfile = profileGen->getVelocityProfile();

    // Assert that position and velocity profiles are not empty
    TEST_ASSERT_FALSE(positionProfile.empty());
    TEST_ASSERT_FALSE(velocityProfile.empty());

    // Assert that the first and last position values are reasonable
    TEST_ASSERT_EQUAL_FLOAT(0.0f, positionProfile.front());
    TEST_ASSERT_EQUAL_FLOAT(totalDistance, positionProfile.back());

    // Assert that velocity values are within range
    for (float velocity : velocityProfile) {
        TEST_ASSERT_GREATER_THAN(0.0f, velocity);
    }
}


int main() {
    UNITY_BEGIN();  // Initialize Unity

    // Run the tests
    RUN_TEST(test_generateScurveProfile);

    UNITY_END();  // Finalize Unity

    return 0;
}
