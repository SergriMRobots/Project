#include "controline.h"

void ControlLine::Init()
{
  connect(&slider, &QSlider::valueChanged, this, &ControlLine::slider_value_changed);
  connect(&spinbox, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, &ControlLine::spinbox_value_changed);

  connect(&slider, &QSlider::sliderReleased, this, &ControlLine::slider_released);

  connect(&checkBox, &QCheckBox::toggled, this, &ControlLine::checkbox_changed);
  connect(&button, &QPushButton::clicked, this, &ControlLine::zero_button_clicked);

  connect(&decrementKey, &HotKey::button_pressed, this, &ControlLine::decrement_button_pressed);
  connect(&incrementKey, &HotKey::button_pressed, this, &ControlLine::increment_button_pressed);

  connect(&decrementKey, &HotKey::button_released, this, &ControlLine::decrement_button_released);
  connect(&incrementKey, &HotKey::button_released, this, &ControlLine::increment_button_released);
}

void ControlLine::spinbox_value_changed(double newValue)
{
  if (checkBox.checkState() != Qt::CheckState::Checked)
  {
    value = newValue;
    int width = slider.maximum() - slider.minimum();
    int newInt = static_cast<int>((1 + newValue) * (width / 2));
    disconnect(&slider, &QSlider::valueChanged, this, &ControlLine::slider_value_changed);
    slider.setValue(newInt);
    connect(&slider, &QSlider::valueChanged, this, &ControlLine::slider_value_changed);
  }
  else
  {
    disconnect(&spinbox, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this,
               &ControlLine::spinbox_value_changed);
    spinbox.setValue(0.0);
    connect(&spinbox, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, &ControlLine::spinbox_value_changed);
  }
}

void ControlLine::slider_value_changed(int newValue)
{
  double width = static_cast<double>(slider.maximum() - slider.minimum());
  value = 2 * (newValue / width) - 1;
  disconnect(&spinbox, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, &ControlLine::spinbox_value_changed);
  spinbox.setValue(value);
  connect(&spinbox, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, &ControlLine::spinbox_value_changed);
}

void ControlLine::slider_released()
{
  if (checkBox.checkState() == Qt::CheckState::Checked)
  {
    SetValue(0);
  }
}

void ControlLine::zero_button_clicked()
{
  SetValue(0);
}

void ControlLine::checkbox_changed(bool newState)
{
  if (newState)
  {
    SetValue(0.0);
  }
}

void ControlLine::decrement_button_pressed()
{
  SetValue(value - 0.1);
}

void ControlLine::increment_button_pressed()
{
  SetValue(value + 0.1);
}

void ControlLine::decrement_button_released()
{
  if (checkBox.checkState() == Qt::CheckState::Checked)
  {
    SetValue(0);
  }
}

void ControlLine::increment_button_released()
{
  if (checkBox.checkState() == Qt::CheckState::Checked)
  {
    SetValue(0);
  }
}

ControlLine::ControlLine(HotKeyList& hotkeyList, QSlider& slider, QDoubleSpinBox& spinbox, QPushButton& button,
                         QCheckBox& checkBox, QLineEdit& decriment, QLineEdit& incriment)
  : decrementKey(hotkeyList, decriment)
  , incrementKey(hotkeyList, incriment)
  , slider(slider)
  , spinbox(spinbox)
  , button(button)
  , checkBox(checkBox)
{
  Init();
}

ControlLine::ControlLine(const ControlLine& a)
  : slider(a.slider)
  , spinbox(a.spinbox)
  , button(a.button)
  , checkBox(a.checkBox)
  , decrementKey(a.decrementKey)
  , incrementKey(a.incrementKey)
{
  Init();
}

ControlLine& ControlLine::operator=(const ControlLine& a)
{
  Init();
}

void ControlLine::SetValue(double newValue)
{
  if ((-1.0 <= newValue) && (newValue <= 1.0))
  {
    disconnect(&slider, &QSlider::valueChanged, this, &ControlLine::slider_value_changed);
    disconnect(&spinbox, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this,
               &ControlLine::spinbox_value_changed);
    value = newValue;
    int width = slider.maximum() - slider.minimum();
    int newInt = static_cast<int>((1 + value) * (width / 2));
    slider.setValue(newInt);
    spinbox.setValue(value);
    connect(&slider, &QSlider::valueChanged, this, &ControlLine::slider_value_changed);
    connect(&spinbox, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, &ControlLine::spinbox_value_changed);
  }
}
