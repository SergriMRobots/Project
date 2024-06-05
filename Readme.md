**Task 1** 

**Кратко**
 Наиболее оптимальным для наших задач я вижу подход с virtual forces, для того, чтобы решить проблему аппроксимации будущего положения можно сперва попробовать простейший (как в пункте 3), а, если будет недостаточно -- прикручивать mpc. Для ускорения вычислений в данных статьях используют некоторые структуры данных: octree\графы, кроме них каких-либо подходов оптимизации не обнаружено. Для рассчета расстояний везде используют алгоритм GJK, а для детекторирования столкновений -- библиотеку FCL


**literature overview**

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

   6. [Model predictive control of a collaborative manipulator considering dynamic obstacles](https://onlinelibrary.wiley.com/doi/full/10.1002/oca.2599)

      ***Что делают:** используют MPC для избежания столкновений статических и динамических препятствий
      
      ***Подход:*** Они внедряют MPC контроллер. Для решения задачи оптимизации они используют графовую структуру и ссылаются на *Rösmann*. Также они приводят разбор алгоритма управления в зависимости от времени и показывают, на какие масштабы времени стоит обратить внимание
      
      ***Что используют:*** Графовую структуру для ускорения вычислений при решении задачи оптимизации
      
      ***Плюсы:***
      
      ***Минусы:***

7. [Real-time (self)-collision avoidance task on a HRP-2 humanoid robot](https://hal-lirmm.ccsd.cnrs.fr/lirmm-00798791/file/2008_icra_stasse-real_time_self_collision_avoidance.pdf)
   
   ***Что делают:**  Алгоритм управления для self collision avoidance, подходящего для контроллера на основе угловых скоростей 
   
   ***Подход:*** Утверждают, что для устойчивого управления coll.avoidance. необходима выпуклая оболочка virtual volumes на роботе, которая обеспечит непрерывность градиенита расстояния между точками. Выводят руккурентные уравнения для joints velocity, и затем они ссылаются на [Khatib 1986](https://journals.sagepub.com/doi/10.1177/027836498600500106) (раздел joint limit avoidance)  и пишут закон упрправления, основанный на градиенте cost function
   
   ***Что используют:*** Gradient Projection Method (один из методов решения оптимизациионной задачи)
   
   ***Плюсы:*** пока не ясны, но результаты показывают, что self colision avoidance работает
   
   ***Минусы:***

8. [Extensions to Reactive Self-Collision Avoidance for Torque and Position Controlled Humanoids](https://core.ac.uk/download/pdf/11146265.pdf)

   ***Что делают:** Алгоритм управления для self collision avoidance, подходящего для контроллера на основе моментов или позиционного контроллера
   
   ***Подход:*** Artificial potential field
   
   ***Что используют:*** В этой статье интересным сестом является секция "DISTANCE COMPUTATION". **Gilbert–Johnson–Keerthi distance algorithm** важен
   
   ***Плюсы:*** 

   ***Минусы:***

9. [Dynamic collision estimator for collaborative robots: A dynamic Bayesian network with Markov model for highly reliable collision detection](https://www.sciencedirect.com/science/article/pii/S0736584523001679)

   ***Что делают:** детектируют столкновения
   
10. Neural networks (simple)
    https://www.mdpi.com/1424-8220/21/12/4235
    https://jeas.springeropen.com/counter/pdf/10.1186/s44147-023-00214-8.pdf

    
    


  
