#ifndef FlexCAN_T4_manager_h
#define FlexCAN_T4_manager_h
#include <map>
#include <vector>

//シングルトン設計
//シングルトン設計のためインスタンスを呼び出すときはポインタ変数として確保する。
//代入演算子を使用した場合コピーされるのでシングルトンにならないので代入演算子、コピーコンストラクタの使用を禁止にしている。
//使用方法はCANDEBUG.inoを参照のこと
class FlexCAN_T4_manager {
protected:
  FlexCAN_T4_manager();

  bool inited = false;
  // CANbus init時にTrue
  bool can1busisopen = false;
  bool can2busisopen = false;
  bool can3busisopen = false;
  // CAN受信データ途絶時にTrue
  bool flgNoCan1Data = true;
  bool flgNoCan2Data = true;
  bool flgNoCan3Data = true;

  void operator=(const FlexCAN_T4_manager &obj) {}
  FlexCAN_T4_manager(const FlexCAN_T4_manager &obj) {}

  // mapテーブルクリア関数
  void clearAllTable();
  // vectorテーブルクリア関数
  void clearAllVector();

public:
  //インスタンス操作
  static FlexCAN_T4_manager *getInstance();

  // init関連
  void init(bool canbus1, bool canbus2, bool canbus3);
  bool isinit();

  // CAN受信データ途絶検知関数
  bool isNoCan1();
  bool isNoCan2();
  bool isNoCan3();

  void restartCan();

  // CAN1メッセージ操作
  bool CAN1isOpen(); // canがopenしているかどうか
  void readBus1(); // canbusからデータを受け取ってhasmap上に格納する。
  //格納されているデータ(buf)をCANIDを指定してとってくる。第２引数にはデータの送り先を指定する。
  void getBus1(uint32_t canid, uint8_t data[8]);

  // CAN2メッセージ操作
  bool CAN2isOpen();
  void readBus2();
  void getBus2(uint32_t canid, uint8_t data[8]);

  // CAN3メッセージ操作
  bool CAN3isOpen();
  void readBus3();
  void getBus3(uint32_t canid, uint8_t data[8]);

  //使用しているCANbusのデータを受信する関数
  void readAll();

  void pushBus1(uint32_t canid, uint8_t buf[8]);
  void pushBus2(uint32_t canid, uint8_t buf[8]);
  void pushBus3(uint32_t canid, uint8_t buf[8]);
  // CANbusへの送信
  void writeAll();
};
#endif
