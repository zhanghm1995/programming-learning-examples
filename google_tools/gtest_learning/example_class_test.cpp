#include "example_class.h"

#include "gtest/gtest.h"

TEST(ExampleClassTest, STLProcess) {
    ExampleClass example1("google", 50.0);
    EXPECT_EQ(example1.GetMaxInVector(), 39);
    EXPECT_TRUE(example1.IsNumberIn(1));
    EXPECT_FALSE(example1.IsNumberIn(100));
}

TEST(ExampleClassTest, MiscTest) {
    ExampleClass example1("Lee", 78.9);
    // String
    EXPECT_EQ(example1.GetName(), "Lee");
    // Double number
    EXPECT_EQ(example1.height(), 78.9);
    EXPECT_NEAR(example1.height(), 78.9, 1e-5);
    EXPECT_NEAR(example1.height(), 78.89, 1e-5); // will failure
    EXPECT_NEAR(example1.height(), 79.0, 1e-5); // will failure
    EXPECT_DOUBLE_EQ(example1.height(), 78.9);
    EXPECT_FLOAT_EQ(example1.height(), 79.0); // will failure
    EXPECT_FLOAT_EQ(example1.height(), 78.9);
}