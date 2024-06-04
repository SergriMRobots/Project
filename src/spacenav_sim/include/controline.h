#ifndef CONTORLINE_H
#define CONTORLINE_H

#include "hotkey.h"
#include <QSlider>
#include <QDoubleSpinBox>
#include <QPushButton>
#include <QCheckBox>
#include <iostream>

class ControlLine : public QObject
{
  Q_OBJECT
private:
  QSlider& slider;
  QDoubleSpinBox& spinbox;
  QPushButton& button;
  QCheckBox& checkBox;

  double value;
  HotKey decrementKey;
  HotKey incrementKey;
  void Init();
private slots:
  void spinbox_value_changed(double newValue);
  void slider_value_changed(int newValue);
  void slider_released();
  void zero_button_clicked();
  void checkbox_changed(bool newState);
  void decrement_button_pressed();
  void increment_button_pressed();

  void decrement_button_released();
  void increment_button_released();

public:
  ControlLine(HotKeyList& hotkeyList, QSlider& slider, QDoubleSpinBox& spinbox, QPushButton& button,
              QCheckBox& checkBox, QLineEdit& decriment, QLineEdit& incriment);
  ControlLine(const ControlLine& a);
  ControlLine& operator=(const ControlLine& a);

  inline double GetValue()
  {
    return value;
  }
  inline void keyPressed(QChar& key)
  {
    if (incrementKey.get_hotkey() == key)
    {
      incrementKey.set_active(true);
      return;
    }
    if (decrementKey.get_hotkey() == key)
    {
      decrementKey.set_active(true);
      return;
    }
  }
  inline void keyReleased(QChar& key)
  {
    if (incrementKey.get_hotkey() == key)
    {
      incrementKey.set_active(false);
      return;
    }
    if (decrementKey.get_hotkey() == key)
    {
      decrementKey.set_active(false);
      return;
    }
  }

  void checkKey()
  {
    incrementKey.check_status();
    decrementKey.check_status();
  }
  void SetValue(double newValue);
};
#endif  // CONTORLINE_H
