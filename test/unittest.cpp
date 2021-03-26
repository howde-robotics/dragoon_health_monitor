#include "myAlgo.h"
#include "gtest/gtest.h"
#include <ros/ros.h>

class UnitTest : public ::testing ::Test
{
protected:
	howde::MyAlgo* algo;
	int x;
	void SetUp() override
	{
		// algo = new howde::MyAlgo(x);
	}
	void TearDown() override
	{
		delete algo;
	}
};

TEST_F(UnitTest, test_case_1)
{
	x = 0;
	algo = new howde::MyAlgo(x);
	int y = 2;
	int output = algo->run(x, y);
	ASSERT_EQ(output, 1);
	delete algo;
}

TEST_F(UnitTest, test_case_2)
{
	x = 0;
	algo = new howde::MyAlgo(x);
	int z = -4;
	int output = algo->someFunction(z);
	ASSERT_EQ(output, -2);
	delete algo;
}

int main(int argc, char** argv)
{
  ::testing::InitGoogleTest(&argc, argv);

  ros::init(argc, argv, "unittest");
	ros::NodeHandle nh;
  return RUN_ALL_TESTS();
}