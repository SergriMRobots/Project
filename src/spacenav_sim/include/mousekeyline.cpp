#include "mousekeyline.h"

MouseKeyLine::MouseKeyLine(HotKeyList& hotkeyList, QPushButton& button, QCheckBox& checkBox, QLineEdit& hotkeyInput)
  : hotkeyList(hotkeyList)
  , button(button)
  , indicator(checkBox)
  , hotkeyInput(hotkeyInput)
  , hotkey(hotkeyList, hotkeyInput)
{
  Init();
}

MouseKeyLine::MouseKeyLine(const MouseKeyLine& a)
  : hotkeyList(a.hotkeyList), button(a.button), indicator(a.indicator), hotkeyInput(a.hotkeyInput), hotkey(a.hotkey)
{
  Init();
}

MouseKeyLine& MouseKeyLine::operator=(const MouseKeyLine& a)
{
  Init();
}

void MouseKeyLine::button_pressed()
{
  value = true;
  indicator.setCheckState(Qt::CheckState::Checked);
}

void MouseKeyLine::button_released()
{
  value = false;
  indicator.setCheckState(Qt::CheckState::Unchecked);
}

void MouseKeyLine::Init()
{
  value = false;
  connect(&hotkey, &HotKey::button_pressed, this, &MouseKeyLine::button_pressed);
  connect(&hotkey, &HotKey::button_released, this, &MouseKeyLine::button_released);
  connect(&button, &QPushButton::pressed, this, &MouseKeyLine::button_pressed);
  connect(&button, &QPushButton::released, this, &MouseKeyLine::button_released);
}
