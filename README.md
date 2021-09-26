# Sequence
Block based sequence description framework.

## Demo codes
### Print&Delay
```
#include "Sequence/inc/sequence.h"
#include <windows.h>

using namespace std;
using namespace seq;

int main()
{
    Sequence sequence
    (
        "Main",
        new block::Print("Hello"),
        new block::Delay(1.0),
        new block::Print("World"),
        new block::Delay(1.0)
        );

    sequence.compile(true);//set block hierarchy. default parameter bool debug=false. provide true to show debug.
    sequence.start();

    while (1) //use timer callback instead
    {
        Sequence::spinOnce();//regularly call in timer callback(with ros::spinOnce() in ROS).
        Sleep(1);//simulates timer loop
    }
}


```
result:
```
Sequence started(Main)
|___Print(Hello)
Hello
|___Delay(1.000000)
|___Print(World)
World
|___Delay(1.000000)
Sequence terminated(Main)
```

### Sequence in sequence
```
#include "Sequence/inc/sequence.h"
#include <windows.h>

using namespace std;
using namespace seq;

int main()
{
    Sequence sequence
    (
        "Main",
        new block::Print("Inner sequence start"),
        new block::SequenceBlock(new Sequence
        (
            "Inner Sequence",
            new block::Print("Hello World"),
            new block::Delay(1.0)
        )),
        new block::Print("Inner sequence end")
    );

    sequence.compile(true);
    sequence.start();

    while (1) //use timer callback instead
    {
        Sequence::spinOnce();
        Sleep(1);
    }
}

```
result:
```
Sequence started(Main)
|___Print(inner sequence start)
Inner sequence start
|___Sequence(Inner Sequence)
| |___Print(Hello World)
Hello World
| |___Delay(1.000000)
| |___Print(Inner sequence end)
Inner sequence end
Sequence terminated(Main)
```
