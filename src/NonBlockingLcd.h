#pragma once

#include <Arduino.h>
#include <Wire.h>

#define LCD_COLS 16
#define LCD_ROWS  2
#define LCD_TOTAL (LCD_COLS * LCD_ROWS)

// HD44780 commands (PCF8574経由の通常1602用)
#define LCD_CMD_CLEARDISPLAY   0x01
#define LCD_CMD_RETURNHOME     0x02
#define LCD_CMD_ENTRYMODESET   0x06  // increment, no shift
#define LCD_CMD_DISPLAYOFF     0x08  // display off
#define LCD_CMD_DISPLAYON      0x0C  // display on, cursor off, blink off
#define LCD_CMD_FUNCTIONSET    0x28  // 4-bit, 2-line, 5x8

// PCF8574 bit mapping
#define LCD_BIT_RS  0x01
#define LCD_BIT_RW  0x02
#define LCD_BIT_EN  0x04
#define LCD_BIT_BL  0x08  // backlight

// ---------------------------------------------------------------------------
// OLED1602 (US2066系, I2C 0x3C) — 下記を移植・非ブロッキング化
//   https://github.com/mkt-kuno/insitu_logger_esp32/blob/develop/src/oled1602.cpp
//   https://github.com/mkt-kuno/insitu_logger_esp32/blob/develop/src/oled1602.h
// 通信形式: [addr][0x80][cmd] / [addr][0x40][data]
// 元コードの delay(100)/delay(10) は millis() ベースの待ちに置換 (pt/非ブロッキング対応)
// ---------------------------------------------------------------------------
#define OLED1602_ADDR      0x3C
#define OLED1602_CMD_MODE  0x80
#define OLED1602_DATA_MODE 0x40
// 元コードの delay 時間をそのまま踏襲 (ブロッキングせず waitUntil_ で待つ)
#define OLED1602_POWER_WAIT_MS 100
#define OLED1602_CMD_WAIT_MS    10
#define OLED1602_FINAL_WAIT_MS 100
#define LCD_SCAN_RETRY_MS 1000

// oled1602.cpp::begin() の sendCommand 列 (最後の 0x0C は別送)
static constexpr uint8_t OLED1602_INIT_CMDS[] = {
  0x2A, 0x71, 0x5C, 0x28,
  0x08, 0x2A, 0x79,
  0xD5, 0x70, 0x78,
  0x08, 0x06,
  0x2A, 0x79,
  0x72, 0x00,
  0xDA, 0x10,
  0x81, 0xFF,
  0xDB, 0x30,
  0xDC, 0x03,
  0x78, 0x28, 0x2A,
  0x06, 0x08, 0x28,
  0x01, 0x80,
};
static constexpr uint8_t OLED1602_INIT_COUNT = sizeof(OLED1602_INIT_CMDS);
static constexpr uint8_t OLED1602_DISPON_CMD = 0x0C;

// ESP32側の使用条件再現: begin()後に setContrast(0x02) → lcdOn() を呼んでいた。
// 移植側のinitテーブルだけではこの2手が抜けるため、init後段として同じ順序で送る。
#define OLED1602_CONTRAST 0xFF
static constexpr uint8_t OLED1602_CONTRAST_CMDS[] = {
  0x2A, 0x79, 0x81, OLED1602_CONTRAST, 0x78, 0x28,
};
static constexpr uint8_t OLED1602_CONTRAST_COUNT = sizeof(OLED1602_CONTRAST_CMDS);

// I2C scan range for PCF8574 / PCF8574A LCD backpacks
// NOTE: 0x3C は OLED1602 として扱うためスキャン対象から除外 (下記 INIT_SCAN で先行判定)
static constexpr uint8_t LCD_SCAN_ADDRS[] = {
  0x20, 0x21, 0x22, 0x23, 0x24, 0x25, 0x26, 0x27,  // PCF8574
  0x38, 0x39, 0x3A, 0x3B, 0x3D, 0x3E, 0x3F,        // PCF8574A (0x3C除く)
};
static constexpr uint8_t LCD_SCAN_COUNT = sizeof(LCD_SCAN_ADDRS);

class NonBlockingLcd {
public:
  // Call once in setup(). Returns immediately.
  // Non-blocking flow:
  //   Wire.begin -> probe 0x3C (OLED1602) -> else scan PCF8574/PCF8574A ->
  //     OLED: US2066 init sequence / PCF: HD44780 init sequence.
  //     if not found: stays inactive (setText/update are no-op).
  void begin() {
    state_ = INIT_WIRE;
    waitUntil_ = millis() + 100;  // power-on settle before touching the bus
    scanPos_ = 0;
    bl_ = LCD_BIT_BL;
    active_ = false;
    addr_ = 0;
    kind_ = KIND_NONE;
    oledIdx_ = 0;
  }

  // No-op if scan found no LCD.
  void setText(uint8_t row, const char* text) {
    if (!active_ || row >= LCD_ROWS) return;
    uint8_t offset = row * LCD_COLS;
    uint8_t i = 0;
    for (; i < LCD_COLS && text[i] != '\0'; i++) {
      back_[offset + i] = text[i];
    }
    for (; i < LCD_COLS; i++) {
      back_[offset + i] = ' ';
    }
  }

  void setChar(uint8_t row, uint8_t col, char ch) {
    if (!active_ || row >= LCD_ROWS || col >= LCD_COLS) return;
    back_[row * LCD_COLS + col] = ch;
  }

  // True if a usable LCD was detected at boot.
  bool isActive() const { return active_; }

  // True if init completed (READY state).
  bool isReady() const { return state_ == READY; }

  // True if the detected device is OLED1602 (0x3C).
  bool isOled() const { return active_ && kind_ == KIND_OLED; }

  // Call every loop iteration. Non-blocking.
  bool update() {
    if (state_ < READY) return initStep();
    if (!active_) return false;
    if (kind_ == KIND_OLED) return updateOled();
    return updatePcf();
  }

private:
  // Device kind
  enum Kind : uint8_t {
    KIND_NONE = 0,
    KIND_PCF,    // PCF8574 + HD44780 系1602
    KIND_OLED,   // OLED1602 (US2066系, 0x3C)
  };

  // Init state machine states
  enum State : uint8_t {
    INIT_WIRE = 0,     // Wire.begin + setClock + set short timeout
    INIT_SCAN,         // 0x3C probe -> OLED, else probe PCF range
    INIT_SCAN_WAIT,    // 未検出時のリトライ待ち (一定周期で INIT_SCAN に戻る)
    INIT_PCF_BL,       // backlight on, wait 1000ms for LCD power-up
    INIT_PCF_8BIT_1,   // first 0x03 write, wait >4.1ms
    INIT_PCF_8BIT_2,   // second 0x03 write, wait >4.1ms
    INIT_PCF_8BIT_3,   // third 0x03 write, wait >150us
    INIT_PCF_4BIT,     // switch to 4-bit mode, wait
    INIT_PCF_FUNCSET,  // function set (0x28)
    INIT_PCF_DISPOFF,  // display off (0x08)
    INIT_PCF_CLEAR,    // clear display (0x01), wait >1.52ms
    INIT_PCF_ENTRYMODE,// entry mode set (0x06)
    INIT_PCF_DISPON,   // display on (0x0C)
    INIT_PCF_HOME,     // return home (0x02), wait >1.52ms + buffer init
    INIT_OLED_PRE,     // power-on settle (100ms, 元delay(100)相当)
    INIT_OLED_CMD,     // init table を1コマンドずつ送信 (各10ms待ち)
    INIT_OLED_POST,    // final wait (100ms) + display on (0x0C)
    INIT_OLED_CONTRAST,// setContrast 相当 (0x2A,0x79,0x81,val,0x78,0x28)
    INIT_OLED_LCDON,   // lcdOn 相当 (0x0C) + buffer init
    READY              // normal operation
  };

  State    state_     = READY;
  Kind     kind_      = KIND_NONE;
  uint8_t  bl_        = LCD_BIT_BL;
  uint32_t waitUntil_ = 0;
  uint8_t  scanPos_   = 0;
  uint8_t  addr_      = 0;   // I2C address discovered by scan
  uint8_t  oledIdx_   = 0;   // OLED init table position
  bool     active_    = false;

  char front_[LCD_TOTAL];
  char back_[LCD_TOTAL];

  // --- Init state machine ---
  bool initStep() {
    // 待ち時間中はバスに触らずに戻る (pt/非ブロッキング対応: delay()不使用)
    if (waitUntil_ && millis() < waitUntil_) return false;
    waitUntil_ = 0;

    switch (state_) {
      case INIT_WIRE:
        Wire.begin();
        Wire.setClock(100000);  // PCF8574 の最大 I2C クロックは 100kHz (OLEDも同速で共用)
        #if defined(WIRE_HAS_TIMEOUT) && WIRE_HAS_TIMEOUT > 0
        Wire.setWireTimeout(1000);  // 1ms — 短め: scan を高速化 + バス固着対策
        #endif
        waitUntil_ = millis() + 50;
        state_ = INIT_SCAN;
        break;

      case INIT_SCAN: {
        // 0x3C は2種類あり得るため読み戻しで判別する:
        //   OLED直結 (US2066系, 書き専用) / PCF8574Aバックパック+KS0066U等のLiquidCrystal系 (読み出し可)
        Wire.beginTransmission(OLED1602_ADDR);
        if (Wire.endTransmission() == 0) {
          addr_ = OLED1602_ADDR;
          active_ = true;
          oledIdx_ = 0;
          if (pcfProbe(OLED1602_ADDR)) {
            kind_ = KIND_PCF;  // KS0066U + PCF8574A 等 → LiquidCrystal系で駆動
            Wire.setClock(100000);  // PCF8574の上限に合わせる (再スキャン時の400kHz残留対策)
            waitUntil_ = millis() + 50;
            state_ = INIT_PCF_BL;
          } else {
            kind_ = KIND_OLED;  // US2066系OLED → 拡張コマンド列で駆動
            Wire.setClock(400000);  // OLEDはFast-mode駆動 (US2066は400kHz可)
            waitUntil_ = millis() + OLED1602_POWER_WAIT_MS;  // 元begin()先頭 delay(100)
            state_ = INIT_OLED_PRE;
          }
          break;
        }
        // 0x3C 以外で PCF8574/PCF8574A を探索
        // NACK は即時、ACK は <1ms、setWireTimeout(1ms) が上限。
        for (uint8_t i = 0; i < LCD_SCAN_COUNT; i++) {
          uint8_t a = LCD_SCAN_ADDRS[i];
          Wire.beginTransmission(a);
          if (Wire.endTransmission() == 0) {
            kind_ = KIND_PCF;
            addr_ = a;
            active_ = true;
            Wire.setClock(100000);  // PCF8574の上限に合わせる
            break;
          }
        }
        if (active_) {
          waitUntil_ = millis() + 50;
          state_ = INIT_PCF_BL;
        } else {
          // 液晶らしきものが無かった — 諦めず一定周期で再スキャン
          // (電源立ち上がり遅れ・接触不良時も loop を止めずに復帰できる)
          kind_ = KIND_NONE;
          waitUntil_ = millis() + LCD_SCAN_RETRY_MS;
          state_ = INIT_SCAN_WAIT;
        }
        break;
      }

      case INIT_SCAN_WAIT:
        // 待ち明け後に再スキャンへ (active_ は false のまま)
        state_ = INIT_SCAN;
        break;

      case INIT_PCF_BL:
        expanderWrite(bl_);
        waitUntil_ = millis() + 1000;  // LCD power-up wait
        state_ = INIT_PCF_8BIT_1;
        break;

      case INIT_PCF_8BIT_1:
        write4bits(0x03 << 4);
        waitUntil_ = millis() + 5;     // >4.1ms
        state_ = INIT_PCF_8BIT_2;
        break;

      case INIT_PCF_8BIT_2:
        write4bits(0x03 << 4);
        waitUntil_ = millis() + 5;     // >4.1ms
        state_ = INIT_PCF_8BIT_3;
        break;

      case INIT_PCF_8BIT_3:
        write4bits(0x03 << 4);
        waitUntil_ = millis() + 1;     // >150us
        state_ = INIT_PCF_4BIT;
        break;

      case INIT_PCF_4BIT:
        write4bits(0x02 << 4);         // switch to 4-bit
        waitUntil_ = millis() + 1;     // wait for mode switch
        state_ = INIT_PCF_FUNCSET;
        break;

      case INIT_PCF_FUNCSET:
        command(LCD_CMD_FUNCTIONSET);   // 4-bit, 2-line, 5x8
        waitUntil_ = millis() + 1;
        state_ = INIT_PCF_DISPOFF;
        break;

      case INIT_PCF_DISPOFF:
        command(LCD_CMD_DISPLAYOFF);    // display off before clear
        waitUntil_ = millis() + 1;
        state_ = INIT_PCF_CLEAR;
        break;

      case INIT_PCF_CLEAR:
        command(LCD_CMD_CLEARDISPLAY);
        waitUntil_ = millis() + 3;     // >1.52ms
        state_ = INIT_PCF_ENTRYMODE;
        break;

      case INIT_PCF_ENTRYMODE:
        command(LCD_CMD_ENTRYMODESET);
        waitUntil_ = millis() + 1;
        state_ = INIT_PCF_DISPON;
        break;

      case INIT_PCF_DISPON:
        command(LCD_CMD_DISPLAYON);
        waitUntil_ = millis() + 1;
        state_ = INIT_PCF_HOME;
        break;

      case INIT_PCF_HOME:
        command(LCD_CMD_RETURNHOME);
        waitUntil_ = millis() + 3;     // >1.52ms
        memset(front_, ' ', LCD_TOTAL);
        memset(back_,  ' ', LCD_TOTAL);
        state_ = READY;
        break;

      case INIT_OLED_PRE:
        // 前段の 100ms 待ちが明けたら init テーブル送出へ
        oledIdx_ = 0;
        state_ = INIT_OLED_CMD;
        break;

      case INIT_OLED_CMD:
        if (oledIdx_ < OLED1602_INIT_COUNT) {
          // 元 sendCommand() 内の delay(10) を waitUntil_ に置換
          // NACK 時は init を捨てて再スキャンに戻る (バス固着・電源不安定対策)
          if (oledSendCommand(OLED1602_INIT_CMDS[oledIdx_]) != 0) {
            kind_ = KIND_NONE;
            active_ = false;
            waitUntil_ = millis() + LCD_SCAN_RETRY_MS;
            state_ = INIT_SCAN_WAIT;
            break;
          }
          oledIdx_++;
          waitUntil_ = millis() + OLED1602_CMD_WAIT_MS;
          // 同ステートに留まり、次回呼び出しで次コマンドを送る
        } else {
          waitUntil_ = millis() + OLED1602_FINAL_WAIT_MS;  // 元begin()末尾 delay(100)
          state_ = INIT_OLED_POST;
        }
        break;

      case INIT_OLED_POST:
        if (oledSendCommand(OLED1602_DISPON_CMD) != 0) {  // **** Turn on Display
          kind_ = KIND_NONE;
          active_ = false;
          waitUntil_ = millis() + LCD_SCAN_RETRY_MS;
          state_ = INIT_SCAN_WAIT;
          break;
        }
        oledIdx_ = 0;
        waitUntil_ = millis() + OLED1602_CMD_WAIT_MS;
        state_ = INIT_OLED_CONTRAST;
        break;

      case INIT_OLED_CONTRAST:
        // ESP32使用条件の setContrast(0x02) 相当。NACK時は再スキャンに戻る。
        if (oledIdx_ < OLED1602_CONTRAST_COUNT) {
          if (oledSendCommand(OLED1602_CONTRAST_CMDS[oledIdx_]) != 0) {
            kind_ = KIND_NONE;
            active_ = false;
            waitUntil_ = millis() + LCD_SCAN_RETRY_MS;
            state_ = INIT_SCAN_WAIT;
            break;
          }
          oledIdx_++;
          waitUntil_ = millis() + OLED1602_CMD_WAIT_MS;
        } else {
          state_ = INIT_OLED_LCDON;
        }
        break;

      case INIT_OLED_LCDON:
        // ESP32使用条件の lcdOn() 相当。これで表示ONが確定する。
        if (oledSendCommand(OLED1602_DISPON_CMD) != 0) {
          kind_ = KIND_NONE;
          active_ = false;
          waitUntil_ = millis() + LCD_SCAN_RETRY_MS;
          state_ = INIT_SCAN_WAIT;
          break;
        }
        waitUntil_ = millis() + OLED1602_CMD_WAIT_MS;
        memset(front_, ' ', LCD_TOTAL);
        memset(back_,  ' ', LCD_TOTAL);
        state_ = READY;
        break;

      default:
        break;
    }
    return false;
  }

  // --- Runtime: PCF8574 + HD44780 (従来通り、1文字/呼び出し) ---
  bool updatePcf() {
    // Diff-scan: find one dirty char and write it
    for (uint8_t i = 0; i < LCD_TOTAL; i++) {
      uint8_t pos = (scanPos_ + i) % LCD_TOTAL;
      if (front_[pos] != back_[pos]) {
        front_[pos] = back_[pos];
        setCursorPcf(pos % LCD_COLS, pos / LCD_COLS);
        writeData(front_[pos]);
        scanPos_ = (pos + 1) % LCD_TOTAL;
        return true;
      }
    }
    scanPos_ = 0;
    return false;
  }

  // --- Runtime: OLED1602 (1文字/呼び出し = setCursor + data の2転送のみ、待ちなし) ---
  bool updateOled() {
    for (uint8_t i = 0; i < LCD_TOTAL; i++) {
      uint8_t pos = (scanPos_ + i) % LCD_TOTAL;
      if (front_[pos] != back_[pos]) {
        front_[pos] = back_[pos];
        oledSetCursor(pos % LCD_COLS, pos / LCD_COLS);
        oledSendData(oledRemap((uint8_t)front_[pos]));
        scanPos_ = (pos + 1) % LCD_TOTAL;
        return true;
      }
    }
    scanPos_ = 0;
    return false;
  }

  // --- OLED1602 low-level (oled1602.cpp 移植) ---
  // 戻り値: Wire.endTransmission() の結果 (0=ACK)。NACK時は呼び出し側で再スキャンに戻る。
  uint8_t oledSendCommand(uint8_t cmd) {
    Wire.beginTransmission(addr_);
    Wire.write(OLED1602_CMD_MODE);
    Wire.write(cmd);
    return Wire.endTransmission();
    // NOTE: 元コードの delay(10) は呼び出し側の waitUntil_ で代替 (非ブロッキング)
  }

  void oledSendData(uint8_t data) {
    Wire.beginTransmission(addr_);
    Wire.write(OLED1602_DATA_MODE);
    Wire.write(data);
    Wire.endTransmission();
  }

  // oled1602.cpp の ROM A->B 対策 (ENABLE_REMAP 相当)
  static uint8_t oledRemap(uint8_t value) {
    switch (value) {
      case '[':  return 0b11111010;
      case ']':  return 0b11111100;
      case '|':  return 0b11111110;
      case '\\': return 0b11111011;
      case '_':  return 0b11000100;
      default:   return value;
    }
  }

  void oledSetCursor(uint8_t col, uint8_t row) {
    static const uint8_t rowOffsets[] = {0x00, 0x40};
    if (row > 1) row = 1;
    oledSendCommand(0x80 | (col + rowOffsets[row]));
  }

  // 0x3C 判別用: PCF8574(A) は読み出し可、US2066系OLEDは読み出し不可。
  // E=0 固定のパターンでエクスパンダの read-back を確認する (LCDバスには無害)。
  // 5V/3.3V どちらの給電でも動作する (バスにデータを残さない)。
  bool pcfProbe(uint8_t a) {
    Wire.beginTransmission(a);
    Wire.write(0xF8);  // E=0,BL=1,data=1111
    if (Wire.endTransmission() != 0) return false;
    if (Wire.requestFrom(a, (uint8_t)1) != 1) return false;
    int v1 = Wire.read();
    Wire.beginTransmission(a);
    Wire.write(0x88);  // E=0,BL=1,data=1000
    if (Wire.endTransmission() != 0) return false;
    if (Wire.requestFrom(a, (uint8_t)1) != 1) return false;
    int v2 = Wire.read();
    // 復帰: E=0・バックライト維持 (以降のinitに影響させない)
    Wire.beginTransmission(a);
    Wire.write(LCD_BIT_BL);
    Wire.endTransmission();
    // データ上位4bitが書いた通り読めればPCF8574系とみなす
    // (P3=BLはTr負荷で読めない個体があるため下位はマスク)
    return ((v1 & 0xF0) == 0xF0) && ((v2 & 0xF0) == 0x80);
  }

  // --- I2C low-level (PCF8574 + HD44780) ---

  void i2cWrite(uint8_t data) {
    Wire.beginTransmission(addr_);
    Wire.write(data);
    Wire.endTransmission();
  }

  void expanderWrite(uint8_t data) {
    i2cWrite(data | bl_);
  }

  void pulseEnable(uint8_t data) {
    expanderWrite(data | LCD_BIT_EN);
    delayMicroseconds(1);   // Enable pulse width >450ns
    expanderWrite(data & ~LCD_BIT_EN);
    delayMicroseconds(50);  // Command execution time
  }

  void write4bits(uint8_t value) {
    expanderWrite(value);
    pulseEnable(value);
  }

  void send(uint8_t value, uint8_t mode) {
    uint8_t hi = value & 0xF0;
    uint8_t lo = (value << 4) & 0xF0;
    write4bits(hi | mode);
    write4bits(lo | mode);
  }

  void command(uint8_t cmd) {
    send(cmd, 0);           // RS=0
  }

  void writeData(uint8_t data) {
    send(data, LCD_BIT_RS); // RS=1
  }

  void setCursorPcf(uint8_t col, uint8_t row) {
    static const uint8_t rowOffsets[] = {0x00, 0x40};
    command(0x80 | (col + rowOffsets[row]));
  }
};
