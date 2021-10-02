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
    ros::init(argc, argv, "mission_control");
    ros::NodeHandle nh;
    ros::Rate loopRate(100);

    Sequence sequence
    (
        "Main Sequence",
        new block::Debug("Hello"),
        new block::Delay(1.0),
        new block::Debug("World"),
        new block::Delay(1.0)
    );
    sequence.compile(true);
    sequence.start();

    while (ros::ok())
    {
        ros::spinOnce();
        Sequence::spinOnce();
        loopRate.sleep();
    }
    return 0;
}


```

Result

```
Sequence started.(Main Sequence)
|___Debug(Hello)
|>  Hello
|___Delay(1.000000)
|___Debug(World)
|>  World
|___Delay(1.000000)
Sequence terminated.(Main Sequence)
```

### Sequence in a sequence

```c++
#include <ros/ros.h>
#include <sequence.h>

using namespace std;
using namespace seq;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "mission_control");
    ros::NodeHandle nh;
    ros::Rate loopRate(100);

    Sequence sequence
    (
        "Main Sequence",
        new block::Debug("Inner sequence start"),
        new block::SequenceBlock(new Sequence
        (
            "Inner Sequence",
            new block::Debug("Sequence in a sequence"),
            new block::Delay(1.0)
        )),
        new block::Debug("Inner sequence end")
    );
    sequence.compile(true);
    sequence.start();

    while (ros::ok())
    {
        ros::spinOnce();
        Sequence::spinOnce();
        loopRate.sleep();
    }
    return 0;
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
| |___Debug(Inner Sequence)
|>  Inner sequence end
Sequence terminated.(Main Sequence)
```

### Broadcast

```c++
#include <ros/ros.h>
#include <sequence.h>

using namespace std;
using namespace seq;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "mission_control");
    ros::NodeHandle nh;
    ros::Rate loopRate(100);

    Sequence sequence
    (
        "Main Sequence",
        new block::Debug("Sequence start"),
        new block::WaitForBroadcast("Broadcast!"),
        new block::Debug("Sequence end")
    );
    sequence.compile(true);
    sequence.start();

    while (ros::ok())
    {
    	static cnt = 0;
    	if(cnt++ == 10) Sequence::broadcast("Broadcast!");//broadcast message "Broadcast!" one seconds after start
        ros::spinOnce();
        Sequence::spinOnce();
        loopRate.sleep();
    }
    return 0;
}

```

Result

```
Sequence started.(Main Sequence)
|___Debug(Sequence start)
|>  Sequence start
|___WaitForBroadcast(Broadcast!)
|___Debug(Sequence start)
|>  Sequence end
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
		new block::LoopSequence(new LambdaCondition([]{return false;}), new Sequence
		(
		    new block::Function([&]{sequenceCnt++;}),
		    new block::Delay(1)
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
