**Task 1** 


   ***Что делают:** 
   
   ***Подход:***
   
   ***Что используют:*** 
   ***Плюсы:*** 
   ***Минусы:*** 

literature overview

1. [A Real-Time Reconfigurable Collision Avoidance System for Robot Manipulation](https://doi.org/10.1145/3068796.3068800)
   ***Что делают:** Создают универсальный фреймворк для работы с роботами любой конфигурации для self- и envinroment- collision avoidance.
   
   ***Подход:***(параграф 3.2) Используются обычныые виртуальные объемы (BV -- Bounding Volumes). На кождой итерации рассчитывают forward kinematics (FK) для виртуального двойника, на основе новой позиции смотрят, появились ли коллизии, если нет -- двигают реального робота
   
   ***Что используют:*** [FCL library](https://github.com/flexible-collision-library/fcl) для Distance computation и Collision detection

   ***Плюсы:*** надежность, простота
   
   ***Минусы:*** Слишком *дорого* считать. Если у нас неизвестна траектория (teleoperation mode), то рассчеты придется делать часто.

2. [Model-predictive Collision Avoidance in Teleoperation of Mobile Robots](https://macsphere.mcmaster.ca/bitstream/11375/13318/1/fulltext.pdf)

   ***Что делают:** Создают фреймворк для комбинирования телеопераций с автономным помощником, основанном на MPC. 
   
   ***Подход:*** С помощью данных с робота и линейной экстраполяции команд оператора заглядывают в будущее и производят корректировки траектории, параллельно с этим, ограничивая свободу оператора через feedback систему (мешая оператору)  
   
   ***Что используют:*** MPC
   
   ***Плюсы:*** Смотрит в будущее

   ***Минусы:***  дорого считать, статические препятствия, предопределенная траектория, необходимость haptics feedback системы

3. [NAOqi Framework](http://doc.aldebaran.com/2-1/naoqi/motion/reflexes-collision-avoidance.html)

   ***Что делают:**  main software that runs on the robot and controls it.
   
   ***Подход:*** Самый-самый простой способ предсказания наличия коллизий в будущем
```math
\triangle t \cdot (\vec{n} J)\dot{q} < Dist
```
   
   ***Что используют:*** NAOqi Framework
   
   ***Плюсы:*** простота
   
   ***Минусы:*** необходимо всё равно вычислять якобианы и расстояния всех со всеми

4.  [Real-Time Robot Arm Collision Avoidance System](https://pec99.tripod.com/papers/4_00134270.pdf)

     ***Что делают:** Безопасную систему для телеопераций в режиме реального времени
     
     ***Подход:*** Они сохраняют робота в некой структуре данных, называемой octree, которая, если погуглить, позволяет свести некоторые вычисления с $O(N^2)$ до $O(log(N))$
     
     ***Что используют:*** N-object octree data sructure
     
     ***Плюсы:*** Статья 1992-го года, а значит наши вычислительные мощности много больше. Утверждается, что не требует предопределенной траектории
     
     ***Минусы:***  Я пока не понял, как работать с этой структурой данных

   5. [On-line collision avoidance for collaborative robot manipulators by adjusting off-line generated paths: An industrial use case](https://hal.science/hal-02362167/file/LISPEN_RAS_2019_BEAREE.pdf)

      ***Что делают:** on-line collision avoidance into a real industrial application implementing typical sensors and a commonly used collaborative industrial manipulator.
      
      ***Подход:*** Немного модифицированный метод виртуальных сил (repulsion/attrraction)
      
      ***Что используют:***
      
      ***Плюсы:*** Можно работать с движущимися препятствиями, прост в использовании
      
      ***Минусы:*** не заглядывает в будущее

   6. 
