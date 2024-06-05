**Task 1** 

literature overview

1. [A Real-Time Reconfigurable Collision Avoidance System for Robot Manipulation](https://doi.org/10.1145/3068796.3068800)
   ***Что делают:** Создают универсальный фреймворк для работы с роботами любой конфигурации для self- и envinroment- collision avoidance.
   
   ***Подход:***(параграф 3.2) Используются обычныые виртуальные объемы (BV -- Bounding Volumes). На кождой итерации рассчитывают forward kinematics (FK) для виртуального двойника, на основе новой позиции смотрят, появились ли коллизии, если нет -- двигают реального робота
   
   ***Что используют:*** [FCL library](https://github.com/flexible-collision-library/fcl) для Distance computation и Collision detection

   ***Плюсы:*** надежность, простота
   ***Минусы:*** Слишком *дорого* считать. Если у нас неизвестна траектория (teleoperation mode), то рассчеты придется делать часто.
