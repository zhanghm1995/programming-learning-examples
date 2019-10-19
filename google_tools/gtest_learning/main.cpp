#include <vector>
#include <gtest/gtest.h>

using namespace std;

bool IsEven(int x)
{
    return !(x % 2);
}

TEST(EvenTest, EvenRight) {
    EXPECT_TRUE(IsEven(2));
}