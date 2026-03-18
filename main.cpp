#include <iostream>
using namespace std;

#include "Motor.h"
#include "driver.h"


int main(int argc, char **argv)
{
    int ret = 0;

    int m_mode = 0;
    int m_run = 0;

    EthercatDriver ec_driver;
    Motor myMotor;
    
    cout << "엔코더 스펙을 입력해주세요(단 0보다 큰 값): ";
    cin >> enc;
    cout << "기어비 스펙을 입력해주세요(단 0보다 큰 값): ";
    cin >> gear;
//    cout<<"관절 범위 스펙을 입력해주세요(단 최대 180도): ";
//    cin >> jnt;



    ec_driver.Init_Master();
    ec_driver.Config_MS();
    ec_driver.Activate_Master_Domain();
    ec_driver.Set_Priority();
    ec_driver.Lock_Memory();

    cout << "모터 동작 모드를 선택하세요." << endl;
    cout << "(1): CSP mode (2): CSV mode (3): CST mode" << endl;
    cin >> m_mode;

    switch(m_mode)
    {
        case 1: ec_driver.(모드 뭐였지? driver.h 참고하기) = 0x08;
        case 2: ec_driver. = 0x09;
        case 3: ec_driver. = 0x0A;
    }


    cout << "동작을 시작할까요?" << endl;
    cout << "(1): yes (2): no" << endl;

    cin >> m_run;
    if(m_run==1)
    {
        ec_driver.Init_ec_loop();
    }
    


    



    return ret;
}