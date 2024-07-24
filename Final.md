Отчет о работе и как всем этим пользоваться.


**Предыстория**
Главной задачей было внедрение в проект системы избежания столкновений, которая помогала бы в режиме реального времени в телеуправляемом режиме сохранить в целостности оборудование при условии возможных ошибок оператора (грубо говоря, не давать роботу сталкиваться с объектами при управлении джойстиком).

**Как в общем работают алгоритмы избежания коллизий**
 1. Изначально строится AllowedCollisionMatrix -- матрица коллизий, которая говорит, между какими объектами потенциально могут произойти столкновения, а между какими нет. Это призвано снизить вычислительные затраты.
 2. Различными алгоритмами вычисляются расстояния между объектами, в соответствии с матрицей коллизий. Например Bullet использует сразу несколько алгоритмов в зависимости от формы объекта [link](https://github.com/bulletphysics/bullet3/blob/master/docs/Bullet_User_Manual.pdf) (стр. 16)
 3. Дальше в зависимости от расстояния мы уже можем судить, есть у нас коллизии или нет


В проекте существовали решения, отличающееся от сырого [moveit_servo_vanila](https://github.com/moveit/moveit/tree/1.1.14/moveit_ros/moveit_servo) , в виде файлов *stop_distance_collision_check.cpp*, *threshold_distance_check.cpp* и модифицированного *collision_check.cpp*. Данные решения не удовлетворяли нашим требованиям по скорости или (в случае *stop_distance*) функциональности.

Основной механизм работы кода -- домножение существующих скоростей на некий коэффициент *collision_scale*. Это происходит в файле *servo_calcs*
```
void ServoCalcs::applyVelocityScaling(Eigen::ArrayXd& delta_theta, double singularity_scale)
{
  ...
  double collision_scale = collision_checker_ ? collision_checker_->getScaleCoef(original_joint_state_, future_state) : 1.0;
  ...
}
```

Сущность *collision_checker* выбирается в этом же файле выше в соответствии с параметром *collision_check_type* в файле ur_config.yaml 

```
if (parameters_.check_collisions)
  {
    if (parameters_.collision_check_type == "threshold_distance")
       ...
    else if (parameters_.collision_check_type == "bullet_collision")
       ...
 }
```
По сути это и есть объект класса *CollisionCheck*, от которого наследуются все ответвления в виде *StopDistanceCollisionCheck*, *ThresholdDistanceCollisionCheck* и *BulletCollisionCheck*.

Во всех трех ответвлениях главная функция, которая должна быть реализована это *calcIteration*, которая возвращает число типа double -- коэффициент, на который будет домножаться скорость.
Эта функция вызывается в родительском методе 

``double CollisionCheck::getScaleCoef(const sensor_msgs::JointState& now, const sensor_msgs::JointState& future)
{
  if (use_collision_check)
  {
    ...
    auto scale = calcIteration(now, future); 
    ...
    return scale;
  }`

```







