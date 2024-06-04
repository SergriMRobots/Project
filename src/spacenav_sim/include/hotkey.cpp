#include "hotkey.h"

QChar HotKey::get_symbol(const QString& string)
{
  if ((!string.isEmpty()) && (!(string.size() > 1)))
    return string[0];
  else
  {
    return '\0';
  }
}

bool HotKey::in(QChar& hotkey, HotKeyList& parent)
{
  return (std::find(parent.begin(), parent.end(), hotkey) != parent.end());
}

bool HotKey::in_hotkey_list(QChar& hotkey, HotKeyList& parent)
{
  return ((hotkey == '\0') || in(hotkey, parent));
}

HotKey::HotKey(HotKeyList& parent, QLineEdit& inputField) : parent(parent), inputField(inputField)
{
  button = get_symbol(inputField.text());
  connect(&inputField, &QLineEdit::textEdited, this, &HotKey::try_new_hotkey);
  if (!in_hotkey_list(button, parent))
  {
    parent.push_back(button);
  }
}

HotKey::HotKey(const HotKey& a) : parent(a.parent), inputField(a.inputField), button(a.button)
{
  connect(&inputField, &QLineEdit::textEdited, this, &HotKey::try_new_hotkey);
}

HotKey HotKey::operator=(const HotKey& a)
{
  return HotKey(a);
}

void HotKey::set_active(bool newState)
{
  pressed = newState;
  if (!pressed)
  {
    emit button_released();
  }
}

void HotKey::check_status()
{
  if (pressed)
  {
    emit button_pressed();
  }
}

void HotKey::try_new_hotkey(QString newHotKey)
{
  QChar symbol = get_symbol(newHotKey);
  if (!in_hotkey_list(symbol, parent))
  {
    button = symbol;
    parent.push_back(button);
  }
  else if (button != '\0')
  {
    disconnect(&inputField, &QLineEdit::textEdited, this, &HotKey::try_new_hotkey);
    inputField.setText(button);
    connect(&inputField, &QLineEdit::textEdited, this, &HotKey::try_new_hotkey);
  }
  else
  {
    disconnect(&inputField, &QLineEdit::textEdited, this, &HotKey::try_new_hotkey);
    inputField.setText("");
    connect(&inputField, &QLineEdit::textEdited, this, &HotKey::try_new_hotkey);
  }
}
