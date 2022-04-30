#include <Arduino.h>
#include <SPI.h>
#include <MFRC522.h>
#include <LowPower.h>

#define RST_PIN 9
#define SS_PIN 10 //SDA
MFRC522 mfrc522(SS_PIN, RST_PIN); // 创建新的RFID实例
MFRC522::MIFARE_Key key;
const int open_pin = 4;
const int opsub_pin = 5;
const int btn_in_pin = A1;

const int check_count = 100;
int check_tick = 0;
bool lost_card = false;
byte card_status;

bool inter_open=false;

unsigned long time_start, QR_time_last=0;

byte accepted_uid[][5] = {
    {0x61, 0x89, 0x05, 0x4E},
    {0x8F, 0x4F, 0x3A, 0x88},
    {0x61, 0x89, 0x1D, 0x25},
    {0x61, 0x88, 0x62, 0x8F},
    {0xB2, 0xBE, 0x92, 0x1B},
    {0x61, 0x89, 0x3F, 0x90}};

String accepted_QRid[] = {
    "21214762",
    "21214763"
};

void open_door()
{
    //Serial.println("open");
    digitalWrite(open_pin, HIGH);
    delay(3000);
    digitalWrite(open_pin, LOW);
}

void weak_up_open(){
    time_start = millis();
    inter_open=true;
}

void test1(){
    Serial.begin(9600);
    while(true){
        digitalWrite(opsub_pin, LOW);
        delay(2000);
        digitalWrite(opsub_pin, HIGH);
        delay(2000);
    }
}

void setup()
{
    pinMode(open_pin, OUTPUT);
    pinMode(opsub_pin, OUTPUT);
    pinMode(btn_in_pin, INPUT_PULLUP);
    pinMode(LED_BUILTIN, OUTPUT);

    digitalWrite(LED_BUILTIN, LOW);

    pinMode(3,INPUT);
    digitalWrite(3, HIGH);

    digitalWrite(opsub_pin, HIGH);

    attachInterrupt(1, weak_up_open, LOW);

    //test1();

    time_start = millis();

    Serial.begin(9600); // 设置串口波特率为9600
    SPI.begin();        // SPI开始
    mfrc522.PCD_Init(); // Init MFRC522 card
    mfrc522.PCD_WriteRegister(mfrc522.CommandReg, mfrc522.PCD_NoCmdChange);
}

/**
   将字节数组转储为串行的十六进制值
*/
void dump_byte_array(byte *buffer, byte bufferSize)
{
    for (byte i = 0; i < bufferSize; i++)
    {
        Serial.print(buffer[i] < 0x10 ? " 0" : " ");
        Serial.print(buffer[i], HEX);
    }
}

bool comp_uid(byte *b1, byte *b2, byte size)
{
    for (byte i = 0; i < size; i++)
    {
        if (b1[i] != b2[i])
            return false;
    }
    return true;
}

byte PICC_status()
{
    byte bufferATQA[2];
    byte bufferSize = sizeof(bufferATQA);

    // Reset baud rates
    mfrc522.PCD_WriteRegister(mfrc522.TxModeReg, 0x00);
    mfrc522.PCD_WriteRegister(mfrc522.RxModeReg, 0x00);
    // Reset ModWidthReg
    mfrc522.PCD_WriteRegister(mfrc522.ModWidthReg, 0x26);

    byte result = mfrc522.PICC_RequestA(bufferATQA, &bufferSize);
    return result;
}

void loop()
{
    //delay(50);

    if (inter_open){
        detachInterrupt(1);
        delay(5);

        open_door();

        inter_open=false;

        digitalWrite(opsub_pin, HIGH);
        Serial.begin(9600); // 设置串口波特率为9600
        SPI.begin();        // SPI开始
        mfrc522.PCD_Init(); // Init MFRC522 card
        mfrc522.PCD_WriteRegister(mfrc522.CommandReg, mfrc522.PCD_NoCmdChange);
        attachInterrupt(1, weak_up_open, LOW);
        
        return;
    }

    if(millis()-time_start > 20000){
        digitalWrite(opsub_pin, LOW);
        mfrc522.PCD_WriteRegister(mfrc522.CommandReg, mfrc522.PCD_NoCmdChange | 0x10);

        Serial.end();
        SPI.end();
        //pinMode(0, OUTPUT);
        //pinMode(1, OUTPUT);

        delay(5);
        LowPower.powerDown(SLEEP_FOREVER, ADC_OFF, BOD_OFF);
        detachInterrupt(1);
        return;
    }

    //二维码开门
    if(Serial.available()){
        String str = Serial.readStringUntil('\r');
        for(int i=0;i<2;i++){
            if(str.equals(accepted_QRid[i])){
                open_door();
                break;
            }
        }
        return;
    }

    // 寻找新卡
    card_status = PICC_status();
    if (!(card_status==mfrc522.STATUS_OK || card_status==mfrc522.STATUS_COLLISION))
    {
        // Serial.println("没有找到卡");
        //按钮开门
        if (digitalRead(btn_in_pin) == LOW)
            open_door();

        if (card_status == mfrc522.STATUS_ERROR)
            lost_card = true;

        check_tick++;
        if (check_tick >= check_count)
        {
            if (lost_card && card_status == mfrc522.STATUS_TIMEOUT)
            {
                mfrc522.PCD_Init();
                lost_card = false;
            }
            check_tick = 0;
        }

        return;
    }

    // 选择一张卡
    if (!mfrc522.PICC_ReadCardSerial())
    {
        //Serial.println("没有卡可选");
        return;
    }

    bool can_open = false;
    for (int i = 0; i < 6; i++)
    {
        if (comp_uid(mfrc522.uid.uidByte, accepted_uid[i], mfrc522.uid.size))
        {
            can_open = true;
            break;
        }
    }

    if (can_open)
        open_door();

    MFRC522::StatusCode status;
    if (status != MFRC522::STATUS_OK)
    {
        //Serial.print(F("身份验证失败？或者是卡链接失败"));
        //Serial.println(mfrc522.GetStatusCodeName(status));
        return;
    }
    //停止 PICC
    mfrc522.PICC_HaltA();
    //停止加密PCD
    mfrc522.PCD_StopCrypto1();
}