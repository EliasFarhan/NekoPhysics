#include "physics/physics.h"

#include <gtest/gtest.h>

// Checksums detect rollback desyncs, so equal or swapped fields must not cancel out (review S1.7)

TEST(Checksum, SameBodySameChecksum)
{
    neko::Body a{};
    a.position = { neko::Scalar{1.5f}, neko::Scalar{-2.0f} };
    const neko::Body b = a;
    EXPECT_EQ(neko::GenerateChecksum(a), neko::GenerateChecksum(b));
}

TEST(Checksum, EqualFieldsDoNotCancel)
{
    neko::Body a{};
    a.position = { neko::Scalar{1}, neko::Scalar{1} };
    const neko::Body b{};
    EXPECT_NE(neko::GenerateChecksum(a), neko::GenerateChecksum(b));

    neko::Body c{};
    c.position = { neko::Scalar{2}, neko::Scalar{3} };
    c.velocity = c.position;
    EXPECT_NE(neko::GenerateChecksum(c), neko::GenerateChecksum(b));
}

TEST(Checksum, SwappedFieldsDiffer)
{
    neko::Body a{};
    a.position = { neko::Scalar{1}, neko::Scalar{2} };
    neko::Body b{};
    b.position = { neko::Scalar{2}, neko::Scalar{1} };
    EXPECT_NE(neko::GenerateChecksum(a), neko::GenerateChecksum(b));

    neko::Body c{};
    c.velocity = a.position;
    EXPECT_NE(neko::GenerateChecksum(a), neko::GenerateChecksum(c));
}

TEST(Checksum, ColliderSwappedOffsetDiffers)
{
    neko::Collider a{};
    a.offset = { neko::Scalar{1}, neko::Scalar{2} };
    neko::Collider b{};
    b.offset = { neko::Scalar{2}, neko::Scalar{1} };
    EXPECT_NE(neko::GenerateChecksum(a), neko::GenerateChecksum(b));
}

TEST(Checksum, AabbSwappedHalfSizeDiffers)
{
    neko::AabbCollider a{};
    a.halfSize = { neko::Scalar{1}, neko::Scalar{2} };
    neko::AabbCollider b{};
    b.halfSize = { neko::Scalar{2}, neko::Scalar{1} };
    EXPECT_NE(neko::GenerateChecksum(a), neko::GenerateChecksum(b));
}

int main(int argc, char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
