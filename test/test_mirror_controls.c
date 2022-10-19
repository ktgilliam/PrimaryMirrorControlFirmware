#include <unity.h>
//#include <primary_mirror_ctrl.h>


void setUp(void) {
    // set stuff up here
}

void tearDown(void) {
    // clean stuff up here
}

void test_home(void){

    home(2);

    TEST_ASSERT_EQUAL_INT(1,1);
}

void test_move_relative(void) {

    
    
}

int main( int argc, char **argv) {
    UNITY_BEGIN();



    UNITY_END();
}