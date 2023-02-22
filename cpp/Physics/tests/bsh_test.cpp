#include "bsh/bsh.h"

#include <gtest/gtest.h>
#include <gmock/gmock.h>

class MockBSH : public neko::BoundingSurfaceHierarchy
{
public:
    MOCK_METHOD(void, Insert, (const neko::ColliderAabb&));
    MOCK_METHOD(void, CalculatePairs, ());
    MOCK_METHOD(void, Clear, () );
    MOCK_METHOD(void, SetWorldAabb, (const neko::Aabbf&) );
    MOCK_METHOD( const std::vector<neko::TriggerPair>&, GetPossiblePairs,(), (const));
};

struct BSHMockTest : public ::testing::Test
{
    MockBSH bsh;

    BSHMockTest()
    {
        EXPECT_CALL(bsh, Clear);
        EXPECT_CALL(bsh, Insert);
        EXPECT_CALL(bsh, CalculatePairs);
        EXPECT_CALL(bsh, GetPossiblePairs);
    }

};