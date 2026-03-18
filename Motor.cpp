#include <iostream>
#include "Motor.h"
#include <cmath>
using namespace std;



bool Motor::InitMember(int enc, int gear, float jnt, float pos)
{
    if(enc<0){
        cout<<"0보다 큰 엔코더 값을 입력해주세요"<<endl;
        return false;
    }
    else if(gear<0){
        cout<<"0보다 큰 기어비 값을 입력해주세요"<<endl;
        return false;
    }
    else if(jnt>180 || jnt<-180){
        cout<<"180보다 작거나 같은 관절 값을 입력해주세요."<<endl;
        cout<<"입력된 관절 값은 시계/반시계 방향 모두에 적용되므로"<<endl;
        cout<<"최대 값인 180도 입력 시 최대 회전 범위는 한 바퀴가 됩니다"<<endl;
        return false;
    }
    else if(pos>jnt || pos<-jnt){
        cout<<"방위의 범위는 -"<<jnt<<"부터 +"<<jnt<<"까지 입니다"<<endl;
        return false;
    }
    else{
        encoder    = enc;
        gear_ratio = gear;
        joint      = jnt;
        position   = pos;
        return true;
    }
}

int Motor::GetCnt() const
{
    int cnt;
    cnt = (position/360)*encoder*gear_ratio;
    
    return cnt;
}


// int main(void)
// {
//     int enc, gear;
//     float jnt, pos;

//     Motor myMotor;

//     cout<<"엔코더 스펙을 입력해주세요(단 0보다 큰 값): ";
//     cin >> enc;
//     cout<<"기어비 스펙을 입력해주세요(단 0보다 큰 값): ";
//     cin >> gear;
//     cout<<"관절 범위 스펙을 입력해주세요(단 최대 180도): ";
//     cin >> jnt;
//     while(1)
//     {
//         cout<<"위치를 입력해주세요: ";
//         cin >> pos;
//         cout<<" "<<endl;

//         if (myMotor.InitMember(enc, gear, jnt, pos))
//         {
//             const int cnt = myMotor.GetCnt();

//             if (cnt>0){
//                 cout<<"cnt 값은 "<<cnt<<", 방향은 반시계 방향입니다."<<endl;
//                 cout<<" "<<endl;
//             }
//             else if(cnt<0){
//                 cout<<"cnt 값은 "<<-cnt<<", 방향은 시계 방향입니다."<<endl;
//                 cout<<" "<<endl;
//             }
//             else{
//                 cout<<"cnt 값은 0, 회전하지 않았습니다"<<endl;
//                 cout<<" "<<endl;
//             }
//         }
//     } 
//     return 0;
// }