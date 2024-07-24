Отчет о работе и как всем этим пользоваться.


**Предыстория**
Главной задачей было внедрение в проект системы избежания столкновений, которая помогала бы в режиме реального времени в телеуправляемом режиме сохранить в целостности оборудование при условии возможных ошибок оператора (грубо говоря, не давать роботу сталкиваться с объектами при управлении джойстиком).

**Как в общем работают алгоритмы избежания коллизий**
 1. Изначально строится AllowedCollisionMatrix -- матрица коллизий, которая говорит, между какими объектами потенциально могут произойти столкновения, а между какими нет. Это призвано снизить вычислительные затраты.
 2. Различными алгоритмами вычисляются расстояния между объектами, в соответствии с матрицей коллизий. Например Bullet использует сразу несколько алгоритмов в зависимости от формы объекта [link](https://github.com/bulletphysics/bullet3/blob/master/docs/Bullet_User_Manual.pdf) (стр. 16) или "Bullet_UserManual.pdf" в папке. 
 3. Дальше среди этих расстояний выбирается минимальное и в зависимости от него мы уже можем судить, есть у нас коллизии или нет.


В проекте существовали решения, отличающееся от сырого [moveit_servo_vanila](https://github.com/moveit/moveit/tree/1.1.14/moveit_ros/moveit_servo) , в виде файлов *stop_distance_collision_check.cpp*, *threshold_distance_check.cpp* и модифицированного *collision_check.cpp*. Данные решения не удовлетворяли нашим требованиям по скорости или (в случае *stop_distance*) функциональности. После анализа кода и существующих решений было принято решение о попытке ускорения метода *threshold_distance* путем внедрения библиотеки bullet. 

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

```double CollisionCheck::getScaleCoef(const sensor_msgs::JointState& now, const sensor_msgs::JointState& future)
{
  if (use_collision_check)
  {
    ...
    auto scale = calcIteration(now, future); 
    ...
    return scale;
  }

```

**Bullet** 
1. Почему Bullet
2. Как реализован код
3. Какие подводные

   1. Bullet производит быстрое вычисление расстояний. Тесты показали, что в сравнении с *threshold_distance*, изначально взятой из FCL, Bullet в среднем был в 30 раз быстрее, что позволяет нам уложиться в 125 Hz для применения его в режиме реального времени.
   2. Собственно, Америку я не открывал и просто использовал [туториал](https://moveit.ros.org/bullet/collision%20detection/moveit/2020/11/18/bullet-collision.html) и [github](https://github.com/moveit/moveit/issues/2998). Конфигурацию файлов и настроек я сделал ровно такую же, как и у *threshold_distance*: то есть буквально делал поиск по файлам строчки "thresholddistance" и тд... и рядом добавлял настройки для своего класса Bullet, реализованного в */src/moveit_servo/src/bullet_collision.cpp*

      2.2 Для спинки кресла надо было сформировать файлы в папке *tms_ur_description* папку *chair*  и добавить

      ```
      <xacro:include filename="$(find tms_ur_description)/addons/chair/macros/chair_macro.xacro"/>
      ...
      <xacro:add_chair/>
      ```
      в файл */home/mrobots/next2/tms_ws_lite/src/tms_ur_description/urdf/ur.xacro*

      
   4. Подводные оказались в том, что bullet плохо работает с невыпуклыми формами, поэтому сложные фигуры лучше или аппроксимировать более простыми или (как в случае со спинкой кресла) разбивать на меньшие части и собирать отдельно в urdf файле (смотри */src/tms_ur_description/addons/chair/macros/chair_macro.xacro*). То есть **невыпуклых форм в виде готовых stl файлов надо избегать**. Это оказалась известная проблема, но чтобы ее найти, потребовалось много времени [link](https://github.com/bulletphysics/bullet3/issues/1507) [link2](https://github.com/bulletphysics/bullet3/issues/2531)  






