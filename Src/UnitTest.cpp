/**
******************************************************************************
* @file    UnitTest.cpp
* @author  Serbay Ozkan
* @version V1.0.0
* @date    14-October-2019
* @brief   Unit Test Example App. Source File
******************************************************************************
*/

#include "CppUTest/TestHarness.h"

class addClass
{
	private:
		int sumTotal;
	public:
		addClass(void): sumTotal {0} { } 	
		int getTotal(void);
		void add(int, int);
};

void addClass::add(int value1, int value2)
{
	sumTotal = value1 + value2;
}

int addClass::getTotal(void)
{
	return sumTotal;
}

TEST_GROUP(myFirstUnitTest)
{
	
};

TEST(myFirstUnitTest, API1)
{
		FAIL("I am Failed!");
}

TEST(myFirstUnitTest, API2)
{
		STRCMP_EQUAL("Hello", "Hello");
}

TEST(myFirstUnitTest, API3)
{
		addClass adder;
		adder.add(100, 200);

		CHECK_EQUAL (310, adder.getTotal());
}
