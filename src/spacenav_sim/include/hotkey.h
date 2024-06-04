#ifndef HOTKEY_H
#define HOTKEY_H
#include <QObject>
#include <QLineEdit>
#include <vector>

class HotKey;
typedef std::vector<QChar> HotKeyList;

class HotKey : public QObject
{
  Q_OBJECT

  HotKeyList& parent;
  QLineEdit& inputField;
  QChar button;
  bool pressed = false;

public:
  static QChar get_symbol(const QString& string);
  inline static bool in(QChar& hotkey, HotKeyList& parent);
  inline static bool in_hotkey_list(QChar& hotkey, HotKeyList& parent);

  HotKey(HotKeyList& parent, QLineEdit& inputField);
  HotKey(const HotKey& a);
  HotKey operator=(const HotKey& a);
  void set_active(bool newState);
  QChar get_hotkey()
  {
    return button;
  }
  void check_status();
private slots:

  void try_new_hotkey(QString newHotKey);

signals:
  void button_pressed();
  void button_released();
};
#endif  // HOTKEY_H
