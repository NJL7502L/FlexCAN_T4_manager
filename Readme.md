# FlexCAN_T4_manager

## FlexCAN_T4_managerの概要
Teensy4.0で行われる全てのCAN通信の送受信を管理する\
全てのCAN通信はこのライブラリを通して行われる。
そのためこのライブラリはシングルトンである。

## 各関数の説明(public関数のみ)
```c++
static FlexCAN_T4_manager *getInstance()
```
インスタンスのアドレスを返す.
FlexCAN_T4_managerにアクセスする時に唯一インスタンスにアクセスすることができる関数。基本的に他のクラスでこれを呼び出してアロー演算子を使用してこのクラスのメンバ関数にアクセスする。


```c++
void init(bool canbus1,bool canbus2,bool canbus3)
```
初期化関数.
引数はそれぞれのcanbusを使用するかどうかをtrue/falseで指定する.


```c++
bool isinit()
```
canbusの初期化が完了している場合trueを返す。


```c++
bool isNoCan1()
bool isNoCan2()
bool isNoCan3()
```
メッセージの受信に一定回数以上失敗した場合trueを返す


```c++
void restartCan()
```
canbusをリセットしたい時に使用する。


```c++
bool CAN1isOpen()
bool CAN2isOpen()
bool CAN3isOpen()
```
それぞれのcanbusが開通しているかどうかを返す。


```c++
void getCAN1mes()
void getCAN2mes()
void getCAN3mes()
```
それぞれのcanbusからデータを受信してCANIDをキー、CAN_message_tを値としたmapに格納する。


```c++
getAllCANData()
```
使用しているすべてのcanbusのデータを受信する。


```c++
void getCan1Data(uint32_t canid,uint8_t data[8])
void getCan2Data(uint32_t canid,uint8_t data[8])
void getCan3Data(uint32_t canid,uint8_t data[8])
```
格納されているデータをCANIDを指定して取得する。第２引数にデータの格納先を指定する。


```c++
void setrowCAN1Message(uint32_t canid, uint8_t buf[8])
void setrowCAN2Message(uint32_t canid, uint8_t buf[8])
void setrowCAN3Message(uint32_t canid, uint8_t buf[8])
```
第１引数に識別子を指定し、第２引数で指定したデータをvectorに格納する。


```c++
void sendAllCANdata()
```
使用しているCANbusにvectorのデータを全て送信する。


## 使い方
### setupより前に書く

```c++
#include <FlexCAN_T4_manager.h>
FlexCAN_T4_manager* canmanager = FlexCAN_T4_manager::getInstance()
```
getInstance()がFlexCAN_T4_managerのインスタンスのアドレスを返すので、
返されたアドレスをポインタ変数としてcanmanagerに格納する。


### setupに書く
```c++
canmanager->init(true,true,true);
```
initにtrue/falseを渡して、使用するCANbusを指定する。

### データを受信したいとき
```c++
uint32_t canid = 0;
uint8_t data[8] = {};
canmanager->getAllCANdata();
canmanager->getCan1Data(canid,data);
```

### データを送信したいとき
```c++
uint32_t canid = 0;
uint8_t buf[8] = {};
canmanager->setrowCAN1Message(canid,buf);
canmanager->sendAllCANdata();
```

### 再始動させたいとき
```c++
canmanager->restartCan();
```
