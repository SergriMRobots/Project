#ifndef MOUSEKEYLINE_H
#define MOUSEKEYLINE_H

#include "hotkey.h"
#include <QPushButton>
#include <QCheckBox>

#include <iostream>

class MouseKeyLine : public QObject
{
  Q_OBJECT
private:
  QPushButton& button;
  QCheckBox& indicator;
  QLineEdit& hotkeyInput;
  HotKeyList& hotkeyList;
  bool value;
  HotKey hotkey;

public:
  MouseKeyLine(HotKeyList& hotkeyList, QPushButton& button, QCheckBox& checkBox, QLineEdit& hotkeyInput);
  MouseKeyLine(const MouseKeyLine& a);
  MouseKeyLine& operator=(const MouseKeyLine& a);
private slots:
  void button_pressed();
  void button_released();

private:
  void Init();

public:
  inline void keyPressed(QChar& key)
  {
    if (hotkey.get_hotkey() == key)
    {
      hotkey.set_active(true);
      return;
    }
  }
  inline void keyReleased(QChar& key)
  {
    if (hotkey.get_hotkey() == key)
    {
      hotkey.set_active(false);
      return;
    }
  }
  int GetValue()
  {
    return value ? 1 : 0;
  }
  void checkKey()
  {
    hotkey.check_status();
  }
};

#endif  // MOUSEKEYLINE_H
