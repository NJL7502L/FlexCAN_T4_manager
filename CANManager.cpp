#include <FlexCAN_T4.h>
#include <FlexCAN_T4_manager.h>

// CANインスタンス
FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> Can1;
FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_16> Can2;
FlexCAN_T4<CAN3, RX_SIZE_256, TX_SIZE_16> Can3;

//受信側maptable
std::map<uint32_t, CAN_message_t> CAN1_mestable;
std::map<uint32_t, CAN_message_t> CAN2_mestable;
std::map<uint32_t, CAN_message_t> CAN3_mestable;

//送信側vector
std::vector<CAN_message_t> CAN1_senddata;
std::vector<CAN_message_t> CAN2_senddata;
std::vector<CAN_message_t> CAN3_senddata;

//インスタンスを取得することができる唯一の関数
FlexCAN_T4_manager *FlexCAN_T4_manager::getInstance() {
  static FlexCAN_T4_manager canmanager;
  return &canmanager; // canmanagerのインスタンスのアドレスを返す。
}

FlexCAN_T4_manager::FlexCAN_T4_manager() {}

//初期化関数
void FlexCAN_T4_manager::init(bool canbus1, bool canbus2, bool canbus3) {
  if (canbus1) {
    Can1.begin();
    Can1.setBaudRate(1000000);
    can1busisopen = true;
  }
  if (canbus2) {
    Can2.begin();
    Can2.setBaudRate(1000000);
    can2busisopen = true;
  }
  if (canbus3) {
    Can3.begin();
    Can3.setBaudRate(1000000);
    can3busisopen = true;
  }
  delay(500);
  inited = true;
}

bool FlexCAN_T4_manager::isinit() { return inited; }
// mapテーブルクリア関数
void FlexCAN_T4_manager::clearAllTable() {
  CAN1_mestable.clear();
  CAN2_mestable.clear();
  CAN3_mestable.clear();
}
// vectorテーブルクリア関数
void FlexCAN_T4_manager::clearAllVector() {
  CAN1_senddata.clear();
  CAN2_senddata.clear();
  CAN3_senddata.clear();
}

// canbus存在チェック関数
bool FlexCAN_T4_manager::CAN1isOpen() { return can1busisopen; }
bool FlexCAN_T4_manager::CAN2isOpen() { return can2busisopen; }
bool FlexCAN_T4_manager::CAN3isOpen() { return can3busisopen; }

// CAN受信データ途絶検知関数
bool FlexCAN_T4_manager::isNoCan1() { return flgNoCan1Data; }
bool FlexCAN_T4_manager::isNoCan2() { return flgNoCan2Data; }
bool FlexCAN_T4_manager::isNoCan3() { return flgNoCan3Data; }

//再始動関数
void FlexCAN_T4_manager::restartCan() {
  Can1.reset();
  Can2.reset();
  Can3.reset();
  clearAllTable();
  clearAllVector();
  init(can1busisopen, can2busisopen, can3busisopen);
}

// canbusからデータを受信する関数
void FlexCAN_T4_manager::getCAN1mes() {
  CAN_message_t CAN1msg;
  static int can1Cnt;
  while (Can1.read(CAN1msg)) {
    CAN1_mestable[CAN1msg.id] = CAN1msg;
    can1Cnt = 0;
  }
  // CAN受信エラーカウンタで分岐
  if (can1Cnt == 0) { //正常受信
    flgNoCan1Data = false;
    can1Cnt += 1;
  } else if (can1Cnt < 16) { //エラーだが許容範囲
    can1Cnt += 1;
  } else { // CANデータ受信途絶のとき緊急停止。
    flgNoCan1Data = true;
  }
}

void FlexCAN_T4_manager::getCAN2mes() {
  CAN_message_t CAN2msg;
  static int can2Cnt;
  while (Can2.read(CAN2msg)) {
    CAN2_mestable[CAN2msg.id] = CAN2msg;
    can2Cnt = 0;
  }
  // CAN受信エラーカウンタで分岐
  if (can2Cnt == 0) { //正常受信
    flgNoCan2Data = false;
    can2Cnt += 1;
  } else if (can2Cnt < 16) { //エラーだが許容範囲
    can2Cnt += 1;
  } else { // CANデータ受信途絶のとき緊急停止。
    flgNoCan2Data = true;
  }
}

void FlexCAN_T4_manager::getCAN3mes() {
  CAN_message_t CAN3msg;
  static int can3Cnt;
  while (Can3.read(CAN3msg)) {
    CAN3_mestable[CAN3msg.id] = CAN3msg;
    can3Cnt = 0;
  }
  // CAN受信エラーカウンタで分岐
  if (can3Cnt == 0) { //正常受信
    flgNoCan3Data = false;
    can3Cnt += 1;
  } else if (can3Cnt < 16) { //エラーだが許容範囲
    can3Cnt += 1;
  } else { // CANデータ受信途絶のとき緊急停止。
    flgNoCan3Data = true;
  }
}
// canbusからそれぞれのCAN1~3のデータを受信する
void FlexCAN_T4_manager::getAllCANdata() {
  if (can1busisopen) {
    getCAN1mes();
  }
  if (can2busisopen) {
    getCAN2mes();
  }
  if (can3busisopen) {
    getCAN3mes();
  }
}

void FlexCAN_T4_manager::getCan1Data(uint32_t canid, uint8_t data[8]) {
  CAN_message_t msg = CAN1_mestable[canid];
  memcpy(data, msg.buf, 8);
}
void FlexCAN_T4_manager::getCan2Data(uint32_t canid, uint8_t data[8]) {
  CAN_message_t msg = CAN2_mestable[canid];
  memcpy(data, msg.buf, 8);
}
void FlexCAN_T4_manager::getCan3Data(uint32_t canid, uint8_t data[8]) {
  CAN_message_t msg = CAN3_mestable[canid];
  memcpy(data, msg.buf, 8);
}

// canmessageの生データをvectorに登録
void FlexCAN_T4_manager::setrawCAN1Message(uint32_t canid, uint8_t buf[8]) {
  CAN_message_t msg;
  msg.id = canid;
  for (int i = 0; i < 8; i++) {
    msg.buf[i] = buf[i];
  }
  CAN1_senddata.push_back(msg);
}
void FlexCAN_T4_manager::setrawCAN2Message(uint32_t canid, uint8_t buf[8]) {
  CAN_message_t msg;
  msg.id = canid;
  for (int i = 0; i < 8; i++) {
    msg.buf[i] = buf[i];
  }
  CAN2_senddata.push_back(msg);
}
void FlexCAN_T4_manager::setrawCAN3Message(uint32_t canid, uint8_t buf[8]) {
  CAN_message_t msg;
  msg.id = canid;
  for (int i = 0; i < 8; i++) {
    msg.buf[i] = buf[i];
  }
  CAN3_senddata.push_back(msg);
}

// CANBusにデータを送信
void FlexCAN_T4_manager::sendAllCANdata() {
  // BUS 1
  if (CAN1isOpen()) {
    for (CAN_message_t outmes : CAN1_senddata) {
      Can1.write(outmes);
    }
  }
  CAN1_senddata.clear();
  // BUS 2
  if (CAN2isOpen()) {
    for (CAN_message_t outmes : CAN2_senddata) {
      Can2.write(outmes);
    }
  }
  CAN2_senddata.clear();
  // BUS 3
  if (CAN3isOpen()) {
    for (CAN_message_t outmes : CAN3_senddata) {
      Can3.write(outmes);
    }
  }
  CAN3_senddata.clear();
}
