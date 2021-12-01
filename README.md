# Sequence

Block based sequential mission management framework.

## Demo codes

### Print&Delay

```c++
#include <ros/ros.h>
#include <sequence.h>

using namespace std;
using namespace seq;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "sequence_test_node");
    ros::NodeHandle nh;

    Sequence sequence;
    sequence.compose
    (
        "main",
        make_shared<block::Debug>("Hello"),
        make_shared<block::Delay>(0.5),
        make_shared<block::Debug>("World")
    );
    sequence.compile(true);
    sequence.start();


    ros::Rate loopRate(100);
    while (ros::ok())
    {
        Sequence::spinOnce();
        ros::spinOnce();
        loopRate.sleep();
    }
}


```

Result

```
Sequence started.(main)
|___Debug(Hello)
|>  Hello
|___Delay(0.500000)
|___Debug(World)
|>  World
Sequence terminated.(main)
```

### Sequence in a sequence

```c++
#include <ros/ros.h>
#include <sequence.h>

using namespace std;
using namespace seq;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "sequence_test_node");
    ros::NodeHandle nh;

    Sequence sequence;
    sequence.compose
    (
        "Main Sequence",
        make_shared<block::Debug>("Inner sequence start"),
        make_shared<block::SequenceBlock>(make_shared<Sequence>
        (
            "Inner Sequence",
            make_shared<block::Debug>("Sequence in a sequence"),
            make_shared<block::Delay>(1.0)
        )),
        make_shared<block::Debug>("Inner sequence end")
    );
    sequence.compile(true);
    sequence.start();


    ros::Rate loopRate(100);
    while (ros::ok())
    {
        Sequence::spinOnce();
        ros::spinOnce();
        loopRate.sleep();
    }
}
```

Result

```
Sequence started.(Main Sequence)
|___Debug(Inner sequence start)
|>  Inner sequence start
|___Sequence(Inner Sequence)
| |___Debug(Sequence in a sequence)
| |>  Sequence in a sequence
| |___Delay(1.000000)
|___Debug(Inner sequence end)
|>  Inner sequence end
Sequence terminated.(Main Sequence)
```

### In embedded developments

```c++
//make sure it's a C++ project

/* USER CODE BEGIN Includes */
#include "../../Sequence/inc/sequence.h"
using namespace seq;
...

/* USER CODE BEGIN PV */
int sequenceCnt=0;
...

int main(void)
{
    /* USER CODE BEGIN 1 */
    Sequence sequence
    (
        make_shared<block::LoopSequence>( make_shared<LambdaCondition>([]{return false;}),  make_shared<Sequence>
        (
            make_shared<block::Function>([&]{sequenceCnt++;}),
            make_shared<block::Delay>(1)
        ))
    );
    
    sequence.compile();
    sequence.start();

    /* USER CODE END 1 */
    ...
    
/* USER CODE BEGIN 4 */
    
//100Hz(0.001sec) timer callback
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	Sequence::spinOnce(0.001);
}
...

```
